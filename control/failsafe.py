import os
import time
import threading

_ip = "192.168.159.5"
_ip = "10.42.0.1"
#_ip = "10.42.0.2"

_last_connected_time = 0
_check_connection_thread, _time_lock, _run_flag_lock = None, None, None
_loop_run_flag = False

def init():
    """
    Starts an infinite loop pinging _ip every 0.3 seconds in a new thread. The time of the last successful is saved.
    :return: None
    """
    global _last_connected_time, _time_lock, _check_connection_thread, _loop_run_flag, _run_flag_lock
    if _loop_run_flag:
        print("gps.init() called more than once.")
        return
    _last_connected_time = time.time()
    _time_lock = threading.Lock()
    _run_flag_lock = threading.Lock()
    _check_connection_thread = threading.Thread(target=_check_connection_loop)
    _loop_run_flag = True
    _check_connection_thread.start()

def _check_connection_loop():
    global _last_connected_time
    _run_flag_lock.acquire()
    while _loop_run_flag:
        _run_flag_lock.release()
        if os.system("ping -c 1 -w 1 > /dev/null " + _ip) == 0:  # try once, give up after a second (numbers < 1 second don't work)
            current_time = time.time()
            _time_lock.acquire()
            _last_connected_time = current_time
            _time_lock.release()
            time.sleep(0.3) # Don't ping too often (uses network and processing power)
        _run_flag_lock.acquire()


def get_connection_up():
    """
    Returns if the connection worked within the last 2 seconds. To be called by the main thread.
    :return: Boolean, if the connection worked within the last 2 seconds
    """
    _time_lock.acquire()
    last_time = _last_connected_time
    _time_lock.release()
    return (time.time() - last_time) < 1.5

def check_connection_now():
    """
    Pings the ip address _ip once with a timeout of 1 second. (Blocking up to 1 second)
    :return: True if there is a connection, False otherwise
    """
    if os.system("ping -c 1 -w 1 > /dev/null " + _ip) == 0:
        return True
    return False

def finish():
    """
    Ends the loop and the thread started in init()
    :return: None
    """
    global  _loop_run_flag
    _run_flag_lock.acquire()
    _loop_run_flag = False
    _run_flag_lock.release()
