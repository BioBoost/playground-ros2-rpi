#pragma once

#include <sys/ioctl.h>
#include <unistd.h>
#include <string>
#include <fcntl.h>          //O_RDONLY
#include <stdexcept>
#include <mutex>
#include <linux/joystick.h>

namespace Controllers {

  /*
    * Pointer to write function used for i2c communication.
    * Also see http://linux.die.net/man/2/write
    */
  ssize_t (* _linux_write)(int, const void *, size_t) = write;
  
  /*
    * Pointer to read function used for i2c communication.
    * Also see http://linux.die.net/man/2/read
    */
  ssize_t (* _linux_read)(int, void *, size_t) = read;

  /*
    * Pointer to open function used for i2c communication.
    * Also see https://linux.die.net/man/2/open
    */
  int (* _linux_open)(const char *pathname, int flags, ...) = open;

  /*
    * Pointer to close function used for i2c communication.
    * Also see https://linux.die.net/man/2/open
    */
  int (* _linux_close)(int fd) = close;

  // enum class EventType = { BUTTON, AXIS };
  // enum class ButtonEvent = { NONE, RELEASED, PRESSED };

  // struct GamepadState {
  //   uint8_t id = -1;
  //   ButtonEvent event = ButtonEvent::NONE;
  // };

  class GamepadController {

    public:
      GamepadController(std::string device = "/dev/input/js0") {
        this->device = device;
      }
      
      bool open(void) {
        mutex.lock();
        fileDescriptor = _linux_open(device.c_str(), O_RDONLY);
        mutex.unlock();
        return (fileDescriptor >= 0);
      }

      bool read_event(struct js_event * event) {
        ssize_t bytes;
        mutex.lock();
        bytes = read(this->fileDescriptor, event, sizeof(js_event));
        mutex.unlock();
        return (bytes == sizeof(js_event));
      }

      // uint8_t button_count(void) {
      //   __u8 buttons;
      //   return (ioctl(fileDescriptor, JSIOCGBUTTONS, &buttons) > 0 ? buttons : 0);
      // }

      bool close(void) {
        mutex.lock();
        int result = _linux_close(fileDescriptor);
        fileDescriptor = -1;
        mutex.unlock();
        return (result == 0);
      }

    private:
      std::string device = "/dev/input/js0";
      int fileDescriptor = -1;
      std::mutex mutex;
  };

}


// size_t get_axis_count(int fd)
// {
//     __u8 axes;

//     if (ioctl(fd, JSIOCGAXES, &axes) == -1)
//         return 0;

//     return axes;
// }


// struct axis_state {
//     short x, y;
// };

// /**
//  * Keeps track of the current axis state.
//  *
//  * NOTE: This function assumes that axes are numbered starting from 0, and that
//  * the X axis is an even number, and the Y axis is an odd number. However, this
//  * is usually a safe assumption.
//  *
//  * Returns the axis that the event indicated.
//  */
// size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
// {
//     size_t axis = event->number / 2;

//     if (axis < 3)
//     {
//         if (event->number % 2 == 0)
//             axes[axis].x = event->value;
//         else
//             axes[axis].y = event->value;
//     }

//     return axis;
// }

// int main(int argc, char *argv[])
// {
//     struct js_event event;
//     struct axis_state axes[3] = {0};
//     size_t axis;

//     /* This loop will exit if the controller is unplugged. */
//     while (read_event(js, &event) == 0)
//     {
//         switch (event.type)
//         {
//             case JS_EVENT_BUTTON:
//                 printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
//                 break;
//             case JS_EVENT_AXIS:
//                 axis = get_axis_state(&event, axes);
//                 if (axis < 3)
//                     printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
//                 break;
//             default:
//                 /* Ignore init events. */
//                 break;
//         }
        
//         fflush(stdout);
//     }

//     close(js);
//     return 0;
// }