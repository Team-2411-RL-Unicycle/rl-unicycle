#include <iostream>
#include <iomanip> // For std::setprecision
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>
#include <queue>
#include <pthread.h>
#include <cstring>  // Include for strerror
#include <cerrno>   // Include for errno
#include <csignal>
#include <Python.h>

volatile std::sig_atomic_t signal_caught = 0;



void setThreadPriority() {
    sched_param sch;
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &sch);
    sch.sched_priority = 20; // Set the priority high, but not maximum to avoid starving other threads
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch)) {
        std::cout << "Failed to set Thread scheduling : " << strerror(errno) << std::endl; // Corrected usage of strerror
    }
}

void controlLoop() {
    setThreadPriority();
    std::cout << "RControl Loop Entered" << std::endl;

    PyGILState_STATE gstate;
    std::cout << "Before PyGILState_Ensure()" << std::endl;
    gstate = PyGILState_Ensure();
    std::cout << "After PyGILState_Ensure()" << std::endl;

    std::cout << "Gstate loaded Entered" << std::endl;

    PyObject *pModuleName, *pModule, *pDict, *pClassRobotSystem, *pInstanceRobotSystem, *pResult;   

    pModuleName = PyUnicode_FromString("main");
    pModule = PyImport_Import(pModuleName);
    Py_DECREF(pModuleName);  // Release reference to pModuleName
    std::cout << "Module loaded" << std::endl;

    if (pModule != NULL) {
        pDict = PyModule_GetDict(pModule);  // Borrowed reference, no need to DECREF
        pClassRobotSystem = PyDict_GetItemString(pDict, "RobotSystem");  // Borrowed reference

        std::cout << "Robotsys loaded..." << std::endl;
        if (PyCallable_Check(pClassRobotSystem)) {
            pInstanceRobotSystem = PyObject_CallObject(pClassRobotSystem, NULL);
            
            // Your loop setup...

            while (!signal_caught && pInstanceRobotSystem != NULL) {
                // Your loop logic...

                std::cout << "Looping..." << std::endl;
                // pResult = PyObject_CallMethod(pInstanceRobotSystem, "loop_callback", NULL);
                // if (pResult == NULL) PyErr_Print();
                // else Py_DECREF(pResult);

                // More loop logic...
            }

            Py_DECREF(pInstanceRobotSystem);
        } else PyErr_Print();

        Py_DECREF(pModule);
    } else PyErr_Print();

    PyGILState_Release(gstate);
}

void signal_handler(int signal) {
    signal_caught = 1;
}

int main() {
    Py_Initialize();
    if (!Py_IsInitialized()) {
        std::cerr << "Failed to initialize Python interpreter." << std::endl;
        return -1;
    }

    std::signal(SIGINT, signal_handler);
    std::thread controlThread(controlLoop);
    controlThread.join();

    Py_Finalize();
    return 0;
}
