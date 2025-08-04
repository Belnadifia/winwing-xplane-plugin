#ifndef USBCONTROLLER_H
#define USBCONTROLLER_H

#include <vector>
#include <functional>
#include <string>
#include "usbdevice.h"

#if APL
#include <IOKit/usb/IOUSBLib.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/hid/IOHIDManager.h>
typedef IOHIDManagerRef HIDManagerHandle;
typedef IOHIDDeviceRef HIDDeviceHandle;
#elif IBM
#include <windows.h>
#include <hidsdi.h>
#include <setupapi.h>
#include <dbt.h>
typedef void* HIDManagerHandle;
typedef HANDLE HIDDeviceHandle;
#elif LIN
#include <libudev.h>
typedef struct udev_monitor* HIDManagerHandle;
typedef int HIDDeviceHandle;
#endif

class USBController {
private:
    HIDManagerHandle hidManager;
    
    USBController();
    ~USBController();
    static USBController* instance;
    
    void enumerateDevices();
    
#if APL
    static void DeviceAddedCallback(void *context, IOReturn result, void *sender, IOHIDDeviceRef device);
    static void DeviceRemovedCallback(void *context, IOReturn result, void *sender, IOHIDDeviceRef device);
    bool deviceExistsWithHIDDevice(IOHIDDeviceRef device);
#elif IBM
    void checkForDeviceChanges();
    void enumerateHidDevices(std::function<void(HANDLE, const std::string&)> deviceHandler);
    USBDevice* createDeviceFromHandle(HANDLE hidDevice);
    bool deviceExistsWithHandle(HANDLE hidDevice);
    void addDeviceFromHandle(HANDLE hidDevice);
#elif LIN
    static void DeviceAddedCallback(void *context, struct udev_device *device);
    static void DeviceRemovedCallback(void *context, struct udev_device *device);
    void monitorDevices();
    USBDevice* createDeviceFromPath(const std::string& devicePath);
    bool deviceExistsAtPath(const std::string& devicePath);
    void addDeviceFromPath(const std::string& devicePath);
#endif

public:
    std::vector<USBDevice *> devices;
    static USBController* getInstance();
    void destroy();
    
    bool allProfilesReady();
    void connectAllDevices();
    void disconnectAllDevices();
};

#endif
