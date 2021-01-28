#include <iostream>
#include <cstdio>
#include <fcntl.h>
#include <linux/joystick.h>
#include "include/NewCar.h"

#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0f)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0f / M_PI)

#define JS_PATH "/dev/input/js0"

int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/**
 * Current state of an axis.
 */
struct axis_state
{
    short x, y;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

int main(int argc, char *argv[])
{
    NewCar car;
    int16_t setV = 0;
    float setW = 0;
    int16_t setV_add = 0;
    float setW_add = 0;

    int js;
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    system("clear");
    car.setCarParams(0, 0);

    js = open(JS_PATH, O_RDONLY);

    if (js == -1)
    {
        perror("Could not open joystick");
        exit(0);
    }

    while (read_event(js, &event) == 0)
    {
        switch (event.type)
        {
        case JS_EVENT_BUTTON:
            printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
            break;
        case JS_EVENT_AXIS:
            axis = get_axis_state(&event, axes);
            if (axis < 3)
                printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
            switch (axis)
            {
            case 0:
                setV = (float)axes[axis].y / 32767.0 * 300.0;
                setW = (float)axes[axis].x / 32767.0 * 1.5;
                break;
            case 1:
                setV_add = (float)axes[axis].y / 32767.0 * 200.0;
                setW_add = (float)axes[axis].x / 32767.0 * 1.0;
                break;
            case 2:
                break;
            }

            break;
        default:
            /* Ignore init events. */
            break;
        }

        fflush(stdout);
        car.setCarParams(setV + setV_add, setW + setW_add);
    }

    close(js);

    return 0;
}
