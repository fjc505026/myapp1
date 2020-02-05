/*
 * interpolation.h
 *
 *  Created on: 23Jan.,2020
 *      Author: jingchen
 */

#ifndef INTERPOLATION_H_
#define INTERPOLATION_H_

#include <stdint.h>
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols,\
                      float *dest, uint8_t dest_rows, uint8_t dest_cols);

#endif /* INTERPOLATION_H_ */
