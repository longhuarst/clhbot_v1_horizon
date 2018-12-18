#ifndef __cy_b_h
#define __cy_b_h






#include <stdint.h>
#include <stdbool.h>








typedef struct {
	float x;
	float y;
	float z;
	
	float x_current;
	float y_current;
	float z_current;
	
	float dg1_current;
	float dg2_current;

}cy_b_type;









extern cy_b_type cy_b_;




void cy_b_move_abs(float x_,float y_,float z_);







void cy_b_init(void);
void cy_b_polling(void);








#endif //__cy_b_h
