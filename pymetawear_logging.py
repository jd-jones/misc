# -*- coding: utf-8 -*-
"""
Log data from accelerometer and gyroscope using pymetawear library.

CHANGELOG
---------
2016-11-15: Created by Jonathan D. Jones
"""

import time
import copy

from pymetawear import libmetawear
from pymetawear.client import *
from pymetawear.mbientlab.metawear.core import *
from pymetawear.mbientlab.metawear.processor import *
from pymetawear.mbientlab.metawear.sensor import Accelerometer, AccelerometerBosch, AccelerometerBmi160, GyroBmi160
# (The CPRO devices use the Bosch BMI160 IMU, BMM150 magnetometer, BMP280 pressure + temp)

import numpy as np
from matplotlib import pyplot as plt

class MetawearDevice:
    
    
    def __init__(self, address, sample_accel=True, sample_gyro=True, sample_switch=False):
        
        self.download_finished = False
        self.unknown_entries = {}
        
        self.address = address
        self.client = MetaWearClient(address, backend='pybluez')
        
        self.temp_accel_data = []
        self.temp_accel_times = []
        self.accel_data = []
        self.accel_times = []
        
        self.temp_gyro_data = []
        self.temp_gyro_times = []
        self.gyro_data = []
        self.gyro_times = []
        
        self.switch_data = []
        self.switch_times = []
        
        self.switch_logger_id = None
        self.accel_logger_id = None
        self.gyro_logger_id = None
        self.float_logger_id = None
                
        self.sample_accel = sample_accel
        self.sample_gyro = sample_gyro
        self.sample_switch = sample_switch
        
        # Set up download handler
        progress_update = Fn_Uint_Uint(self.progress_update_handler)
        unknown_entry = Fn_Ubyte_LongLong_ByteArray(self.unknown_entry_handler)
        self.download_handler = LogDownloadHandler(received_progress_update=progress_update,
                received_unknown_entry=unknown_entry,
                received_unhandled_entry=cast(None, Fn_DataPtr))
        
        # Set default accelerometer and gyroscope parameters
        self.set_accel_params()
        self.set_gyro_params()
        
        
    def disconnect(self):
        """
        Disconnect this client from the MetaWear board.
        """
        
        print('Disconnecting from {}...'.format(self.address))
        
        libmetawear.mbl_mw_metawearboard_tear_down(self.client.board)
        time.sleep(1)
        libmetawear.mbl_mw_metawearboard_free(self.client.board)
        time.sleep(1)
        self.client.backend.disconnect()
        time.sleep(1)
    
    
    def set_accel_params(self, sample_rate=25.0, accel_range=2.0):
        
        self.accel_sample_rate = sample_rate
        self.accel_range = accel_range
        
        self.client.accelerometer.set_settings(data_rate=sample_rate,
                                               data_range=accel_range)
        time.sleep(1)
    
    
    def set_gyro_params(self, sample_rate=25.0, gyro_range=125.0):
        
        self.gyro_sample_rate = sample_rate
        self.gyro_range = gyro_range
        
        self.client.gyroscope.set_settings(data_rate=sample_rate,
                                           data_range=gyro_range)
        time.sleep(1)
    
    
    def init_logger(self):
        
        if self.sample_accel:
            accel_signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.client.board)
            self.accel_logger_ready = Fn_VoidPtr(self.accel_logger_ready_handler)
            libmetawear.mbl_mw_datasignal_log(accel_signal, self.accel_logger_ready)
            while self.accel_logger_id is None:
                time.sleep(0.001)
            print('Acceleration logger id: {}'.format(self.accel_logger_id))
        
        if self.sample_gyro:
            gyro_signal = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.client.board)
            self.gyro_logger_ready = Fn_VoidPtr(self.gyro_logger_ready_handler)
            libmetawear.mbl_mw_datasignal_log(gyro_signal, self.gyro_logger_ready)
            while self.gyro_logger_id is None:
                time.sleep(0.001)
            print('Gyroscope logger id: {}'.format(self.gyro_logger_id))
        
        if self.sample_switch:  
            switch_signal = libmetawear.mbl_mw_switch_get_state_data_signal(self.board)
            self.switch_logger_ready = Fn_VoidPtr(self.switch_logger_ready_handler)
            libmetawear.mbl_mw_datasignal_log(switch_signal, self.switch_logger_ready)
            while self.switch_logger_id is None:
                time.sleep(0.001)
        
        # This clears any data left over from previous sessions
        libmetawear.mbl_mw_logging_clear_entries(self.client.board)
    
    
    def init_logger_rss(self):
        
        self.float_logger_ready = Fn_VoidPtr(self.float_logger_ready_handler)
        
        self.accel_signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.client.board)
        self.proc_created = Fn_VoidPtr(self.float_proc_created)
        libmetawear.mbl_mw_dataprocessor_rss_create(self.accel_signal, self.proc_created)
        while self.float_logger_id is None:
            time.sleep(0.001)
        print('Float logger id: {}'.format(self.float_logger_id))
    
    def start_logging(self):
        
        libmetawear.mbl_mw_logging_start(self.client.board, 1)
        self.start_sampling()
    
    
    def stop_logging(self):
        
        libmetawear.mbl_mw_logging_stop(self.client.board)
        self.stop_sampling()
    
    
    def start_sampling(self):
        
        if self.sample_accel:
            self.client.accelerometer.start()
            self.client.accelerometer.toggle_sampling(True)
        
        if self.sample_gyro:
            self.client.gyroscope.start()
            self.client.gyroscope.toggle_sampling(True)
        
        self.start_time = time.time()
    
    
    def stop_sampling(self):
        
        if self.sample_accel:
            self.client.accelerometer.stop()
            self.client.accelerometer.toggle_sampling(False)
        
        if self.sample_gyro:
            self.client.gyroscope.stop()
            self.client.gyroscope.toggle_sampling(False)
        
    
    def download_data(self, num_updates):
        
        print('DOWNLOADING FROM {}'.format(self.address))
        print('-----------------------------------')
        
        # Download logged data; block until the logger has finished downloading
        self.download_finished = False
        libmetawear.mbl_mw_logging_download(self.client.board, num_updates, byref(self.download_handler))
        time_waited = time.time()
        while not self.download_finished:
            time.sleep(0.001)
        time_waited = time.time() - time_waited
        print('Time spent waiting on download: {:.1f} seconds'.format(time_waited))
        
        # Print some info about the downloaded data
        num_samples = len(self.temp_accel_data)
        sample_rate = self.accel_sample_rate
        data_time = num_samples / float(sample_rate)
        recv_time_interval = 0.0
        if self.temp_accel_times:
            recv_time_interval = self.temp_accel_times[-1] - self.temp_accel_times[0]
        fmt_str = 'A | {:.1f} seconds of data ({} samples at {} Hz) downloaded in {:.1f} seconds'
        print(fmt_str.format(data_time, num_samples, sample_rate, recv_time_interval))
        print('A | {} samples previously collected'.format(len(self.accel_data)))
        print('A | {} samples downloaded'.format(len(self.temp_accel_data)))
        
        self.accel_times += self.temp_accel_times
        self.accel_data += self.temp_accel_data
        self.temp_accel_times = []
        self.temp_accel_data = []
        
        # Print some info about the downloaded data
        num_samples = len(self.temp_gyro_data)
        sample_rate = self.gyro_sample_rate
        data_time = num_samples / float(sample_rate)
        recv_time_interval = 0.0
        if self.temp_gyro_times:
            recv_time_interval = self.temp_gyro_times[-1] - self.temp_gyro_times[0]
        fmt_str = 'G | {:.1f} seconds of data ({} samples at {} Hz) downloaded in {:.1f} seconds'
        print(fmt_str.format(data_time, num_samples, sample_rate, recv_time_interval))
        print('G | {} samples previously collected'.format(len(self.gyro_data)))
        print('G | {} samples downloaded'.format(len(self.temp_gyro_data)))
        
        self.gyro_times += self.temp_gyro_times
        self.gyro_data += self.temp_gyro_data
        self.temp_gyro_times = []
        self.temp_gyro_data = []
    
    
    def print_stats(self):
        
        print('\nDEVICE {}'.format(self.address))
        
        num_accel_samples = len(self.accel_data)
        accel_duration = float(num_accel_samples) / self.accel_sample_rate
        accel_time_delta = float(self.accel_data[-1][0] - self.accel_data[0][0]) / 1000.0
        fmt_str = 'A | Downloaded {} samples ({:.2f} seconds @ {:.1f} Hz) spanning {:.2f} seconds'
        print(fmt_str.format(num_accel_samples, accel_duration, self.accel_sample_rate, accel_time_delta))
        
        num_gyro_samples = len(self.gyro_data)
        gyro_duration = float(num_gyro_samples) / self.gyro_sample_rate
        gyro_time_delta = float(self.gyro_data[-1][0] - self.gyro_data[0][0]) / 1000.0
        fmt_str = 'G | Downloaded {} samples ({:.2f} seconds @ {:.1f} Hz) spanning {:.2f} seconds'
        print(fmt_str.format(num_gyro_samples, gyro_duration, self.gyro_sample_rate, gyro_time_delta))
        
    
    def plot_data(self):
        
        # Plot the downloaded data
        if self.accel_data:
            times = np.array([x[0] for x in self.accel_data], dtype=float)
            times -= times[0]
            times /= 1000.0
            samples = np.array([x[1:4] for x in self.accel_data])
            #sample_norms = (samples ** 2).sum(axis=1) ** 0.5
            
            f, axes = plt.subplots(4) #, sharex=True)
            axes[0].plot(times, samples[:,0])
            axes[0].set_ylabel('a_x')
            axes[0].set_title('Downloaded acceleration data [{}]'.format(self.address))
            axes[1].plot(times, samples[:,1])
            axes[1].set_ylabel('a_y')
            axes[2].plot(times, samples[:,2])
            axes[2].set_ylabel('a_z')
            axes[3].plot(times)
            axes[3].set_xlabel('Time (seconds)')
        else:
            print('No acceleration data recorded!')
            
        # Plot the downloaded data
        if self.gyro_data:
            times = np.array([x[0] for x in self.gyro_data], dtype=float)
            times -= times[0]
            times /= 1000.0
            samples = np.array([x[1:4] for x in self.gyro_data])
            #sample_norms = (samples ** 2).sum(axis=1) ** 0.5
            
            f, axes = plt.subplots(4) #, sharex=True)
            axes[0].plot(times, samples[:,0])
            axes[0].set_ylabel('w_x')
            axes[0].set_title('Downloaded angular velocity data [{}]'.format(self.address))
            axes[1].plot(times, samples[:,1])
            axes[1].set_ylabel('w_y')
            axes[2].plot(times, samples[:,2])
            axes[2].set_ylabel('w_z')
            axes[3].plot(times)
            axes[3].set_xlabel('Time (seconds)')
        else:
            print('No angular velocity data recorded!')
        
        plt.show()
    
    
    def set_ble_params(self, min_conn_interval, max_conn_interval, latency, timeout):
        
        libmetawear.mbl_mw_settings_set_connection_parameters(self.client.board,
                                                              min_conn_interval,
                                                              max_conn_interval,
                                                              latency,
                                                              timeout)
                
    
    def progress_update_handler(self, entries_left, total_entries):
        """
        Print download status to the console and notify when download is
        finished.
        """
        
        entries_downloaded = total_entries - entries_left
        samples_downloaded = float(entries_downloaded) / 2.0
        total_samples = float(total_entries) / 2.0
        fmt_str = '{} / {} packets ({:.1f} / {:.1f} samples) downloaded'
        print(fmt_str.format(entries_downloaded, total_entries, samples_downloaded, total_samples))
        if entries_left == 0:        
            self.download_finished = True
    
    
    def unknown_entry_handler(self, ID, epoch, data, length):
        """
        Record unknown log entries and print them to the console.
        """
        
        print('{} | received unknown log entry: id = {}'.format(epoch, ID))
        
        # Log the entry by its ID
        if not ID in self.unknown_entries:
            self.unknown_entries[ID] = []
        data_array = []
        for i in range(length):
            data_array.append(data[i])
        self.unknown_entries[ID].append(data_array)
    
    
    def float_proc_created(self, processor):
        """
        """
        
        print('Processor: {}'.format(processor))
        
        if processor:
            print('Float processor created')
            libmetawear.mbl_mw_datasignal_log(processor, self.float_logger_ready)
        else:
            print('Failed to create float processor')
        
    
    def switch_logger_ready_handler(self, logger):
        """
        Check if logger was created successfully and, if so, subscribe to it
        with the switch data handler.
        """
        
        if logger:
            print('Switch logger ready')
            self.switch_logger_id = libmetawear.mbl_mw_logger_get_id(logger)
            self.switch_data_fn = Fn_DataPtr(self.switch_data_handler)
            libmetawear.mbl_mw_logger_subscribe(logger, self.switch_data_fn)
        else:
            print('Failed to create switch logger')
    
    
    def accel_logger_ready_handler(self, logger):
        """
        Check if logger was created successfully and, if so, subscribe to it
        with the accel float data handler.
        """
        
        if logger:
            print('Acceleration logger ready')
            self.accel_logger_id = libmetawear.mbl_mw_logger_get_id(logger)
            self.accel_data_fn = Fn_DataPtr(self.accel_data_handler)
            libmetawear.mbl_mw_logger_subscribe(logger, self.accel_data_fn)
        else:
            print('Failed to create acceleration logger')
    
    
    def gyro_logger_ready_handler(self, logger):
        """
        Check if logger was created successfully and, if so, subscribe to it
        with the gyro float data handler.
        """
        
        if logger:
            print('Gyroscope logger ready')
            self.gyro_logger_id = libmetawear.mbl_mw_logger_get_id(logger)
            self.gyro_data_fn = Fn_DataPtr(self.gyro_data_handler)
            libmetawear.mbl_mw_logger_subscribe(logger, self.gyro_data_fn)
        else:
            print('Failed to create gyroscope logger')
    
    
    def float_logger_ready_handler(self, logger):
        
        if logger:
            print('Float logger ready')
            self.float_logger_id = libmetawear.mbl_mw_logger_get_id(logger)
            self.float_data_fn = Fn_DataPtr(self.float_data_handler)
            libmetawear.mbl_mw_logger_subscribe(logger, self.gyro_data_fn)
        else:
            print('Failed to create float logger')
    
    
    def switch_data_handler(self, data):
        """
        Record boolean data from switch.
        """
    
        # Cast to pointer to uint32, then dereference pointer, then get the int
        # representation by calling ctypes.c_uint32's 'value' attribute
        contents = copy.deepcopy(cast(data.contents.value, POINTER(c_uint32)).contents)
        switch_pressed = contents.value
        epoch = data.contents.epoch
        
        sample = (epoch, switch_pressed)
        self.switch_data.append(sample)
        self.switch_times.append(time.time())
        
        switch_pressed = contents.value
        if switch_pressed:
            print('{} | STATUS {} | Switch pressed'.format(epoch, switch_pressed))
        else:
            print('{} | STATUS {} | Switch released'.format(epoch, switch_pressed))
    
    
    def accel_data_handler(self, data):
        """
        Record cartesian float data from accelerometer.
        """
        
        contents = copy.deepcopy(cast(data.contents.value, POINTER(CartesianFloat)).contents)
        sample = (data.contents.epoch, contents.x, contents.y, contents.z)
        
        #print('A | {} | {:.3f} {:.3f} {:.3f}'.format(*sample))
        
        self.temp_accel_data.append(sample)
        self.temp_accel_times.append(time.time())
    
    
    def gyro_data_handler(self, data):
        """
        Record cartesian float data from gyroscope.
        """
        
        contents = copy.deepcopy(cast(data.contents.value, POINTER(CartesianFloat)).contents)
        sample = (data.contents.epoch, contents.x, contents.y, contents.z)
        
        #print('G | {} | {:.3f} {:.3f} {:.3f}'.format(*sample))
        
        self.temp_gyro_data.append(sample)
        self.temp_gyro_times.append(time.time())
    
    
    def float_data_handler(self, data):
        """
        """
        
        contents = copy.deepcopy(cast(data.contents.value, POINTER(c_float)).contents)
        sample = (data.contents.epoch, contents.value)
        
        print('{} | {:.3f}'.format(*sample))
    
    
def battery_callback(data):
    """
    Print battery status.
    """

    epoch = data[0]
    battery = data[1]
    print("Battery status: {}%".format(battery[1]))


if __name__ == '__main__':
    
    run_time_mins = 0
    run_time_secs = run_time_mins * 60
    num_notifications = 10
    
    addresses = ('D6:B3:DA:FD:2E:DE',
                 'FC:63:6C:B3:4C:F6',
                 'F7:A1:FC:73:DD:23',
                 'C8:B8:4F:23:CD:E5',
                 'D3:4A:2F:8C:57:5E',
                 'C7:7D:36:B1:5E:7D')
    addresses = addresses[1:5]
    
    # Connect to each device, congifure settings, initialize loggers
    devices = []
    for address in addresses:
        print('\nConnecting to device at {}'.format(address))
        mw = MetawearDevice(address)
        mw.client.battery.notifications(battery_callback)
        mw.client.battery.read_battery_state()
        mw.set_ble_params(7.5, 30.0, 0, 10000)
        #mw.set_accel_params(sample_rate=50.0)
        #mw.set_gyro_params(sample_rate=50.0)
        mw.init_logger_single_axis()
        devices.append(mw)
    
    # Start logging from all devices
    for device in devices:
        device.start_logging()
    init_time = time.time()
    
    # Download data continuously, one device at a time, until the time limit
    # has been reached
    time_delta = time.time() - init_time
    while time_delta < run_time_secs:
        for device in devices:
            print('\nELAPSED TIME: {:.2f} seconds'.format(time_delta))
            device.download_data(num_notifications)
            time_delta = time.time() - init_time
            if time_delta >= run_time_secs:
                break
    
    # Stop logging from all devices
    for device in devices:
        device.stop_logging()
    
    # Perform one final download to catch any remaining samples
    for device in devices:
        print('\nFINAL DOWNLOAD')
        device.download_data(num_notifications)
    
    # Disconnect from all devices
    print('')
    for device in devices:
        device.disconnect()
    
    # Print download stats and plot data
    for device in devices:
        device.print_stats()
        device.plot_data()
    