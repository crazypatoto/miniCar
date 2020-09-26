#include "LedMatrix.h"

LedMatrix::LedMatrix(/* args */)
{
    ws2811_init(&ledstring);  
}

void LedMatrix::setPattern(const uint32_t *pattern)
{
    memcpy(ledstring.channel[0].leds, pattern, 4 * ledstring.channel[0].count);
}
void LedMatrix::render(void)
{
    ws2811_render(&ledstring);
}

void LedMatrix::clear(void){
    memset(ledstring.channel[0].leds, 0, 4 * ledstring.channel[0].count);
    ws2811_render(&ledstring);
}

void LedMatrix::setBrightness(const uint8_t level)
{
    ledstring.channel[0].brightness = level;
    ws2811_render(&ledstring);    
}

LedMatrix::~LedMatrix()
{
}
