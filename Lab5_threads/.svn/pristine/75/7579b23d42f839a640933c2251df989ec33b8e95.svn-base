/*
 * median.c
 *
 *  Created on: 25 May 2017
 *      Author: 12033445
 */


#include "median.h"

uint8_t Median_Filter3(const uint8_t n1, const uint8_t n2, const uint8_t n3)
{
  if (n1 > n2)
  {
    if (n2 > n3)
    {
      return n2;
    }
    else
    {
      if (n1 > n3)
      {
	return n3;
      }
      else
      {
	return n1;
      }
    }
  }
  else
  {
    if (n3 > n2)
    {
      return n2;
    }
    else
    {
      if (n1 > n3)
      {
	return n1;
      }
      else
      {
	return n3;
      }
    }
  }
}
