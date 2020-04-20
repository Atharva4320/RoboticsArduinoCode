# include "Arduino.h"
# include <stdint.h>

# ifndef __SEGMENT_H
# define __SEGMENT_H


struct Segment {
  float motorL;
  float motorR;
  uint32_t dis;
};



# endif
