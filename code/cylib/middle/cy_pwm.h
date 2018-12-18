#ifndef __cy_pwm_h
#define __cy_pwm_h












typedef struct {

	int a;
}cy_pwm_type;









extern cy_pwm_type cy_pwm_;











void cy_pwm_init(void);
void cy_pwm_polling(void);








#endif //__cy_pwm_h
