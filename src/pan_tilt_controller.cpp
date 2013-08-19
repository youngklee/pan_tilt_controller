#include <iostream>
#include <libusb-1.0/libusb.h>
#include "pan_tilt_controller/protocol.h"
#include <time.h>

#include <lcm/lcm-cpp.hpp>
#include "exlcm/position2d_t.hpp"

using namespace std;

bool deviceMatchesVendorProduct(libusb_device *device, unsigned short idVendor, unsigned short idProduct)
{
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(device, &desc);
    return idVendor == desc.idVendor && idProduct == desc.idProduct;
}

void setTarget(int position, int servo)
{
    const unsigned short vendorId = 0x1ffb;
    unsigned short productIDArray[]={0x0089, 0x008a, 0x008b, 0x008c};
    libusb_context *ctx=0;
    libusb_device **device_list=0;
    libusb_init(&ctx);
    int count=libusb_get_device_list(ctx, &device_list);
    for(int i=0;i<count;i++)
    {
        libusb_device *device=device_list[i];
        {
            for(int Id=0;Id<4;Id++)
            {
                if(deviceMatchesVendorProduct(device, vendorId, productIDArray[Id]))
                {
                    libusb_device_handle *device_handle;
                    libusb_open(device, &device_handle);
                    libusb_control_transfer(device_handle, 0x40, REQUEST_SET_TARGET, position*4, servo, 0, 0, (ushort)5000);
                    libusb_close(device_handle);
                    break;
                }
            }
        }
    }
    libusb_free_device_list(device_list, 0);
    libusb_exit(ctx);
}

int16_t x_target, y_target;

class PositionMessageHandler
{
public:
    ~PositionMessageHandler() {}

    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
            const exlcm::position2d_t *msg)
    {
        x_target = msg->position[0];
        y_target = msg->position[1];
    }
};

const int min_angle = 0;
const int max_angle = 180;
const int pan_min_position = 550;
const int pan_max_position = 2550;
const int tilt_min_position = 500;
const int tilt_max_position = 2000;

int angle_to_position(double angle, int min_position, int max_position)
{
    int position = (int)
        (angle - min_angle)/(max_angle - min_angle)*(max_position - min_position)
        + min_position;

    return position;
}

int main()
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    PositionMessageHandler handlerObject;
    lcm.subscribe("target_position", &PositionMessageHandler::handleMessage, &handlerObject);
    
    int img_length = 320;
    int img_height = 240;

    int x_center = img_length/2;
    int y_center = img_height/2;

    int center_threshold = 5;
    int servo_step_size = 5;

    while(1)
    {
        int lcm_fd = lcm.getFileno();
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(lcm_fd, &fds);

        int rfds = select(lcm_fd + 1, &fds, 0, 0, &timeout);

        if (rfds) {
            lcm.handle();
        }
        else {
            int pan_angle, tilt_angle;
            const int pan_servo = 12;
            const int tilt_servo = 13;

            if (x_target < x_center - center_threshold) {
                if (pan_angle >= 5)
                    pan_angle -= servo_step_size;
            }
            else if (x_target > x_center + center_threshold) {
                if (pan_angle <=175)
                    pan_angle += servo_step_size;
            }
            
            if (y_target < y_center - center_threshold) {
                if (tilt_angle >= 5)
                    tilt_angle -= servo_step_size;
            }
            else if (y_target > y_center + center_threshold) {
                if (tilt_angle <=175)
                    tilt_angle += servo_step_size;
            }
            
            int pan_position = angle_to_position(pan_angle, pan_min_position, pan_max_position);
            int tilt_position = angle_to_position(tilt_angle, tilt_min_position, tilt_min_position);

            cout << "Pan, tilt angle: " << pan_angle << tilt_angle << endl;

            setTarget(pan_position, pan_servo);
            setTarget(tilt_position, tilt_servo);

            usleep(20000);
        }
    }
    return 0;
}
