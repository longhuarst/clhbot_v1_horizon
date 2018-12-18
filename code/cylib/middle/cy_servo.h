#ifndef __cy_servo_h
#define __cy_servo_h












typedef struct {

	int a;
}cy_servo_type;









extern cy_servo_type cy_servo_;











void cy_servo_init(void);
void cy_servo_polling(void);



void servo_inner(float dg);
void servo_outter(float dg);

void keep_outter_level(void);


#endif //__cy_servo_h
