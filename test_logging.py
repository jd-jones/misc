# -*- coding: utf-8 -*-
"""
test_logging.py
  test mbientlab metawear logging functionality using python-wrapped C++ API

HISTORY
-------
2016-09-29: Created by Jonathan D. Jones
"""

import time
import copy

from pymetawear import libmetawear
from pymetawear.mbientlab.metawear.core import *
from pymetawear.mbientlab.metawear.sensor import AccelerometerBosch, AccelerometerBmi160
# (The CPRO devices use the Bosch BMI160 IMU, BMM150 magnetometer, BMP280 pressure + temp)

import numpy as np
from matplotlib import pyplot as plt

try:
    from pymetawear.backends.pybluez import PyBluezBackend
except ImportError as e:
    PyBluezBackend = e


# Exception classes
class PyMetaWearException(Exception):
    """
    Main exception.
    """
    pass


class PyMetaWearConnectionTimeout(PyMetaWearException):
    """
    Connection could not be established in time.
    """
    pass


class MetawearDevice:
    
    
    def __init__(self, address, interface='hci0', timeout=None):
        
        self.download_finished = False
        self.unknown_entries = {}
        
        self.address = address
        
        self.prev_time = -1
        self.data_time_offsets = []
        self.logged_data = []
        self.times = []
        
        self.logger_id = None
        
        # Handling of timeout.
        timeout = None
        
        # Setup BLE communication backend and wait for board to initialize
        if isinstance(PyBluezBackend, Exception):
            raise PyMetaWearException(
                "pybluez[ble] package error: {0}".format(PyBluezBackend))
        print('Connecting to {}...'.format(address))
        self.backend = PyBluezBackend(self.address, interface=interface, timeout=timeout, debug=False)
        # block until the board has been initialized
        while (not self.backend.initialized) or \
              (not libmetawear.mbl_mw_metawearboard_is_initialized(self.board)):
            self.backend.sleep(0.001)

        # Check if initialization has been completed successfully.
        print("Initialization status: {}".format(self.backend.initialized))
        print("OK status: {}".format(Status.OK))
        if not self.backend.initialized:
            if self.backend._initialization_status == Status.ERROR_TIMEOUT:
                raise PyMetaWearConnectionTimeout("libmetawear initialization status 16: Timeout")
            else:
                raise PyMetaWearException("libmetawear initialization status {0}".format(
                    self.backend._initialization_status))
        
        accel = libmetawear.mbl_mw_metawearboard_lookup_module(self.board, Module.ACCELEROMETER)
        logging = libmetawear.mbl_mw_metawearboard_lookup_module(self.board, Module.LOGGING)
        print("Accelerometer module: {}".format(accel))
        print("Logging module: {}".format(logging))
        
        # Set up logger
        progress_update = Fn_Uint_Uint(self.progress_update_handler)
        unknown_entry = Fn_Ubyte_LongLong_ByteArray(self.unknown_entry_handler)
        self.download_handler = LogDownloadHandler(received_progress_update=progress_update,
                received_unknown_entry=unknown_entry,
                received_unhandled_entry=cast(None, Fn_DataPtr))
                
        self.logger_ready = Fn_VoidPtr(self.logger_ready_handler)
        
        # Set up data handlers
        self.cartesian_float_data = Fn_DataPtr(self.cartesian_float_data_handler)
        self.data_printer = Fn_DataPtr(self.data_printer_handler)


    @property
    def backend(self):
        """
        The backend object for this client.

        :return: The connected BLE backend.
        :rtype: :class:`pymetawear.backend.BLECommunicationBackend`
        """
        return self.backend


    @property
    def board(self):
        return self.backend.board
        
        
    def disconnect(self):
        """
        Disconnect this client from the MetaWear board.
        """
        
        print('Disconnecting...')
        
        libmetawear.mbl_mw_metawearboard_tear_down(self.board)
        libmetawear.mbl_mw_metawearboard_free(self.board)
        self.backend.disconnect()
    
    
    def test_logger(self):
        """
        Test accelerometer logging capabilities.
        """
        
        print('Pointer to board: {}'.format(self.board))
        
        # some external parameters
        sample_rate = 25.0
        sample_rate_param = AccelerometerBmi160.ODR_25HZ
        num_updates = 1

        # Get accelerometer data signal and configure settings
        acc_signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.board)
        libmetawear.mbl_mw_acc_bosch_set_range(self.board, AccelerometerBosch.FSR_2G)
        libmetawear.mbl_mw_acc_set_odr(self.board, sample_rate_param)
        libmetawear.mbl_mw_acc_write_acceleration_config(self.board)
        
        # Set up logger; block until a logger has been established
        libmetawear.mbl_mw_datasignal_log(acc_signal, self.logger_ready)
        while self.logger_id is None:
            time.sleep(0.001)
        print('logger id: {}'.format(self.logger_id))
        
        # Start logging accelerometer data
        libmetawear.mbl_mw_logging_start(self.board, 1)
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.board)
        libmetawear.mbl_mw_acc_start(self.board)
        self.start_time = time.time()

        # Log data for ~5 seconds
        time.sleep(5)
        
        # Stop logging data
        libmetawear.mbl_mw_logging_stop(self.board)
        libmetawear.mbl_mw_acc_stop(self.board)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(self.board)
        
        # Download logged data; block until the logger has finished downloading
        self.download_finished = False
        libmetawear.mbl_mw_logging_download(self.board, num_updates, byref(self.download_handler))
        wait_time = time.time()
        while not self.download_finished:
            time.sleep(0.001)
        wait_time = time.time() - wait_time
        print('Time spent waiting on download: {:.3} seconds'.format(wait_time))
        
        # Print some info about the downloaded data
        num_samples = len(self.logged_data)
        recv_time_interval = 0
        if self.times:
            recv_time_interval = self.times[-1] - self.times[0]
        fmt_str = '{} seconds of data ({} samples at {} Hz) downloaded in {:.3} seconds'
        print(fmt_str.format(num_samples / float(sample_rate), num_samples, sample_rate, recv_time_interval))
        
        # Plot the downloaded data
        if self.logged_data:
            times = np.array([x[0] for x in self.logged_data], dtype=float)
            times -= times[0]
            times /= 1000.0
            samples = np.array([x[1:4] for x in self.logged_data])
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
            print('No data recorded!')
        
    
    def test_stream(self):
        """
        Test accelerometer streaming capabilities.
        """
        
        # some external parameters
        sample_rate = 25.0
        sample_rate_param = AccelerometerBmi160.ODR_25HZ

        # Get accelerometer data signal and configure settings
        acc_signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.board)
        libmetawear.mbl_mw_datasignal_subscribe(acc_signal, self.cartesian_float_data)
        libmetawear.mbl_mw_acc_bosch_set_range(self.board, AccelerometerBosch.FSR_2G)
        libmetawear.mbl_mw_acc_set_odr(self.board, sample_rate_param)
        libmetawear.mbl_mw_acc_write_acceleration_config(self.board)
        
        # Start sampling
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.board)
        libmetawear.mbl_mw_acc_start(self.board)
        self.start_time = time.time()
        
        # Sample accelerometer for ~5 seconds
        time.sleep(5)
        
        # Stop sampling
        libmetawear.mbl_mw_acc_stop(self.board)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(self.board)

        # Print some info about the downloaded data
        num_samples = len(self.logged_data)
        recv_time_interval = 0
        if self.times:
            recv_time_interval = self.times[-1] - self.times[0]
        fmt_str = '{} seconds of data ({} samples at {} Hz) downloaded in {:.3} seconds'
        print(fmt_str.format(num_samples / float(sample_rate), num_samples, sample_rate, recv_time_interval))
        
        # Plot the downloaded data
        if self.logged_data:
            times = np.array([x[0] for x in self.logged_data], dtype=float)
            times -= times[0]
            times /= 1000.0
            samples = np.array([x[1:4] for x in self.logged_data])
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
            print('No data recorded!')
    
    
    def progress_update_handler(self, entries_left, total_entries):
        """
        Print download status to the console and notify when download is
        finished.
        """
        
        print('Download status: {} / {}'.format(entries_left, total_entries))
        if entries_left == 0:        
            self.download_finished = True
            print('Download finished')
    
    
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
    
    
    def logger_ready_handler(self, logger):
        """
        Check if logger was created successfully and, if so, subscribe to it
        with the cartesian float data handler.
        """
        
        if logger:
            print('logger ready')
            self.logger_id = libmetawear.mbl_mw_logger_get_id(logger)
            libmetawear.mbl_mw_logger_subscribe(logger, self.cartesian_float_data)
        else:
            print('failed to create logger')
        
    
    def cartesian_float_data_handler(self, data):
        """
        Record cartesian float data and print to console.
        """
        
        if (self.prev_time == -1):
            self.prev_time = data.contents.epoch
        else:
            self.data_time_offsets.append(data.contents.epoch - self.prev_time)
            self.prev_time = data.contents.epoch

        contents = copy.deepcopy(cast(data.contents.value, POINTER(CartesianFloat)).contents)
        self.logged_data.append((data.contents.epoch, contents.x, contents.y, contents.z))
        self.times.append(time.time())
        
        elapsed_time = time.time() - self.start_time
        print('{:8.5} | {} | ({:8.5} {:8.5} {:8.5})'.format(elapsed_time, data.contents.epoch, contents.x, contents.y, contents.z))
    
    
    def data_printer_handler(self, data):
        """
        Print cartesian float data to console.
        """
        
        contents = copy.deepcopy(cast(data.contents.value, POINTER(CartesianFloat)).contents)
        print('{} | ({:8.5} {:8.5} {:8.5})'.format(data.contents.epoch, contents.x, contents.y, contents.z))    


if __name__ == '__main__':
    
    addresses = ('D6:B3:DA:FD:2E:DE',
                 'FC:63:6C:B3:4C:F6',
                 'F7:A1:FC:73:DD:23',
                 'C7:7D:36:B1:5E:7D')
    address = addresses[1]
    
    mw = MetawearDevice(address)
    mw.test_logger()
    mw.disconnect()
    
    