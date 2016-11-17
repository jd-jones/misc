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
from pymetawear.mbientlab.metawear.sensor import AccelerometerBosch, AccelerometerBmi160, GyroBmi160
# (The CPRO devices use the Bosch BMI160 IMU, BMM150 magnetometer, BMP280 pressure + temp)

import numpy as np
from matplotlib import pyplot as plt

class MetawearDevice:
    
    
    def __init__(self, addres, sample_accel=True, sample_gyro=True):
        
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
        
        self.accel_logger_id = None
        self.gyro_logger_id = None
                
        self.sample_accel = sample_accel
        self.sample_gyro = sample_gyro
        
        # Set up download handler
        progress_update = Fn_Uint_Uint(self.progress_update_handler)
        unknown_entry = Fn_Ubyte_LongLong_ByteArray(self.unknown_entry_handler)
        self.download_handler = LogDownloadHandler(received_progress_update=progress_update,
                received_unknown_entry=unknown_entry,
                received_unhandled_entry=cast(None, Fn_DataPtr))
        
        
    def disconnect(self):
        """
        Disconnect this client from the MetaWear board.
        """
        
        print('Disconnecting...')
        
        self.client.disconnect()
        time.sleep(1)
    
    
    def set_accel_params(self, sample_rate=25.0, accel_range=2.0):
        
        self.client.accelerometer.set_settings(data_rate=sample_rate,
                                               data_range=accel_range)
        time.sleep(1)
    
    
    def set_gyro_params(self, sample_rate=25.0, gyro_range=125.0):
        
        self.client.gyroscope.set_settings(data_rate=sample_rate,
                                           data_range=gyro_range)
        time.sleep(1)
    
    
    def init_loggers(self):
        
        # Set up logger; block until a logger has been established
        
        if self.sample_accel:
            accel_signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.client.board)
            logger_ready = Fn_VoidPtr(self.accel_logger_ready_handler)
            libmetawear.mbl_mw_datasignal_log(accel_signal, logger_ready)
            while self.accel_logger_id is None:
                time.sleep(0.001)
            print('Acceleration logger id: {}'.format(self.accel_logger_id))
        
        if self.sample_gyro:
            gyro_signal = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.client.board)
            logger_ready = Fn_VoidPtr(self.gyro_logger_ready_handler)
            libmetawear.mbl_mw_datasignal_log(gyro_signal, logger_ready)
            while self.gyro_logger_id is None:
                time.sleep(0.001)
            print('Gyroscope logger id: {}'.format(self.gyro_logger_id))
    
    
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
        
        print('\nDOWNLOAD')
        print('----------')
        
        # Download logged data; block until the logger has finished downloading
        self.download_finished = False
        libmetawear.mbl_mw_logging_download(self.client.board, num_updates, byref(self.download_handler))
        time_waited = time.time()
        while not self.download_finished:
            time.sleep(0.001)
        time_waited = time.time() - time_waited
        print('Time spent waiting on download: {:.3} seconds'.format(time_waited))
        
        # Print some info about the downloaded data
        #num_samples = len(self.temp_accel_data)
        #recv_time_interval = 0.0
        #if self.temp_times:
        #    recv_time_interval = self.temp_accel_times[-1] - self.temp_accel_times[0]
        #fmt_str = '{:.3} seconds of data ({} samples at {} Hz) downloaded in {:.3} seconds'
        #print(fmt_str.format(num_samples / float(sample_rate), num_samples, sample_rate, recv_time_interval))
        
        self.accel_times += self.temp_accel_times
        self.accel_data += self.temp_accel_data
        self.temp_accel_times = []
        self.temp_accel_data = []
        
        # Print some info about the downloaded data
        #num_samples = len(self.temp_gyro_data)
        #recv_time_interval = 0.0
        #if self.temp_times:
        #    recv_time_interval = self.temp_gyro_times[-1] - self.temp_gyro_times[0]
        #fmt_str = '{:.3} seconds of data ({} samples at {} Hz) downloaded in {:.3} seconds'
        #print(fmt_str.format(num_samples / float(sample_rate), num_samples, sample_rate, recv_time_interval))
        
        self.gyro_times += self.temp_gyro_times
        self.gyro_data += self.temp_gyro_data
        self.temp_gyro_times = []
        self.temp_gyro_data = []
    
    
    def plot_data(self):
        
        # Plot the downloaded data
        if self.accel_data:
            times = np.array([x[0] for x in self.accel_data], dtype=float)
            times -= times[0]
            times /= 1000.0
            samples = np.array([x[1:4] for x in self.accel_data])
            sample_norms = (samples ** 2).sum(axis=1) ** 0.5
            
            f, axes = plt.subplots(2, sharex=True)
            axes[0].plot(times, sample_norms)
            axes[0].scatter(times, sample_norms, c='r')
            axes[0].set_ylabel('Acceleration magnitude')
            axes[0].set_title('Downloaded data')
            axes[1].plot(times[1:], np.diff(times))
            axes[1].set_xlabel('Time (seconds)')
            axes[1].set_ylabel('dt (seconds)')
            plt.show()
        else:
            print('No acceleration data recorded!')
            
        # Plot the downloaded data
        if self.gyro_data:
            times = np.array([x[0] for x in self.gyro_data], dtype=float)
            times -= times[0]
            times /= 1000.0
            samples = np.array([x[1:4] for x in self.gyro_data])
            sample_norms = (samples ** 2).sum(axis=1) ** 0.5
            
            f, axes = plt.subplots(2, sharex=True)
            axes[0].plot(times, sample_norms)
            axes[0].scatter(times, sample_norms, c='r')
            axes[0].set_ylabel('Angular velocity magnitude')
            axes[0].set_title('Downloaded data')
            axes[1].plot(times[1:], np.diff(times))
            axes[1].set_xlabel('Time (seconds)')
            axes[1].set_ylabel('dt (seconds)')
            plt.show()
        else:
            print('No angular velocity data recorded!')  
    
    
    def test_logger(self, wait_time, num_downloads, num_updates):
        """
        Test accelerometer logging capabilities.
        """
        
        # configure sensor settings
        self.set_accel_params()
        self.set_gyro_params()
        
        # Set up loggers and download data
        self.init_loggers()        
        self.start_logging()
        print('Waiting {} second(s) while data logs...'.format(wait_time))
        time.sleep(wait_time)
        for i in range(num_downloads):
            self.download_data(num_updates)
        
        # Stop logging data, download any data remaining, plot results
        self.stop_logging()
        self.download_data(num_updates)        
        self.plot_data()
    
    
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
        
        print('{} / {} packets remaining'.format(entries_left, total_entries))
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
    
    
    def accel_logger_ready_handler(self, logger):
        """
        Check if logger was created successfully and, if so, subscribe to it
        with the accel float data handler.
        """
        
        if logger:
            print('acceleration logger ready')
            self.accel_logger_id = libmetawear.mbl_mw_logger_get_id(logger)
            data_handler = Fn_DataPtr(accel_data_handler)
            libmetawear.mbl_mw_logger_subscribe(logger, data_handler)
        else:
            print('failed to create acceleration logger')
    
    
    def gyro_logger_ready_handler(self, logger):
        """
        Check if logger was created successfully and, if so, subscribe to it
        with the gyro float data handler.
        """
        
        if logger:
            print('gyroscope logger ready')
            self.gyro_logger_id = libmetawear.mbl_mw_logger_get_id(logger)
            data_handler = Fn_DataPtr(gyro_data_handler)
            libmetawear.mbl_mw_logger_subscribe(logger, data_handler)
        else:
            print('failed to create gyroscope logger')

    
def accel_data_handler(data):
    """
    Record cartesian float data from accelerometer.
    """
    
    contents = copy.deepcopy(cast(data.contents.value, POINTER(CartesianFloat)).contents)
    sample = (data.contents.epoch, contents.x, contents.y, contents.z)
    
    print('A: {} | {:.3f} {:.3f} {:.3f}'.format(*sample))
    
    #self.temp_accel_data.append(sample)
    #self.temp_accel_times.append(time.time())


def gyro_data_handler(data):
    """
    Record cartesian float data from gyroscope.
    """
    
    contents = copy.deepcopy(cast(data.contents.value, POINTER(CartesianFloat)).contents)
    sample = (data.contents.epoch, contents.x, contents.y, contents.z)
    
    print('G: {} | {:.3f} {:.3f} {:.3f}'.format(*sample))
    
    #self.temp_gyro_data.append(sample)
    #self.temp_gyro_times.append(time.time())
    
        
if __name__ == '__main__':
    
    wait_time = 0.5
    num_reads = 3
    num_notifications = 1
    
    addresses = ('D6:B3:DA:FD:2E:DE',
                 'FC:63:6C:B3:4C:F6',
                 'F7:A1:FC:73:DD:23',
                 'C7:7D:36:B1:5E:7D')
    address = addresses[0]
    
    mw = MetawearDevice(address, sample_accel=True, sample_gyro=True)
    mw.set_ble_params(7.5, 30.0, 0, 100)
    mw.test_logger(wait_time, num_reads, num_notifications)
    time.sleep(1)
    mw.disconnect()
    