#ifndef __cy_led_h
#define __cy_led_h












typedef struct {

	int a;
}cy_led_type;









extern cy_led_type cy_led_;











void cy_led_init(void);
void cy_led_polling(void);








#endif //__cy_led_h
