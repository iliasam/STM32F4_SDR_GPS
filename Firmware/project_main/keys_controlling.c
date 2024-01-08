/* Includes ------------------------------------------------------------------*/
#include "keys_controlling.h"
#include "stm32f4xx_gpio.h"
#include "signal_capture.h" //to get time
#include "main.h"
#include "string.h"
#include "config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//Time in ms
#define KEY_HOLD_TIME            900

//Time in ms
#define KEY_PRESSED_TIME         50

//Time in ms
#define KEY_RELEASE_TIME         50

//Time in ms
#define KEYS_STARTUP_DELAY      500

#define START_TIMER(x, duration)  (x = (signal_capture_get_packet_cnt() + duration))
#define TIMER_ELAPSED(x)  ((signal_capture_get_packet_cnt() > x) ? 1 : 0)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
key_item_t key_up;

uint32_t keys_startup_timer = 0;
uint8_t keys_startup_lock_flag = 1;

/* Private function prototypes -----------------------------------------------*/
uint8_t key_up_presed = 0;

/* Private functions ---------------------------------------------------------*/

void keys_init(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  key_up.gpio_name = BUTTON1_GPIO;
  key_up.pin_name = BUTTON1_PIN;
  keys_functons_init_hardware(&key_up);
  START_TIMER(keys_startup_timer, KEYS_STARTUP_DELAY);
}

void key_handling(void)
{
  keys_functons_update_key_state(&key_up);
  
  if (TIMER_ELAPSED(keys_startup_timer) == 0)
    return; //delay before startup
  else
  {
      keys_startup_lock_flag = 0;
  }
  

  if ((key_up.prev_state == KEY_PRESSED) && 
      (key_up.state == KEY_WAIT_FOR_RELEASE))
  {
    key_up_presed = 1;
  }
  
  if ((key_up.prev_state == KEY_PRESSED) && 
      (key_up.state == KEY_HOLD))
  {

  }
}

//*****************************************************************************

// Initialize single key pin
void keys_functons_init_hardware(key_item_t* key_item)
{
  if (key_item == NULL)
    return;
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Pin = key_item->pin_name;
  GPIO_Init(key_item->gpio_name, &GPIO_InitStructure);
  
  key_item->state = KEY_RELEASED;
}

void keys_functons_update_key_state(key_item_t* key_item)
{
  key_item->prev_state = key_item->state;
  
  if ((key_item->gpio_name->IDR & key_item->pin_name) != 0)
    key_item->current_state = 1;
  else
    key_item->current_state = 0;
  
  if ((key_item->state == KEY_RELEASED) && (key_item->current_state != 0))
  {
    //key presed now
    key_item->state = KEY_PRESSED_WAIT;
    key_item->key_timestamp = signal_capture_get_packet_cnt();
    return;
  }
  
  if (key_item->state == KEY_PRESSED_WAIT)
  {
    uint32_t delta_time = signal_capture_get_packet_cnt() - key_item->key_timestamp;
    if (delta_time > KEY_PRESSED_TIME)
    {
      if (key_item->current_state != 0)
        key_item->state = KEY_PRESSED;
      else
        key_item->state = KEY_RELEASED;
    }
    return;
  }
  
  if ((key_item->state == KEY_PRESSED) || (key_item->state == KEY_HOLD))
  {
    // key not pressed
    if (key_item->current_state == 0)
    {
      key_item->state = KEY_WAIT_FOR_RELEASE;// key is locked here
      key_item->key_timestamp = signal_capture_get_packet_cnt();
      return;
    }
  }
  
  if (key_item->state == KEY_WAIT_FOR_RELEASE)
  {
    uint32_t delta_time = signal_capture_get_packet_cnt() - key_item->key_timestamp;
    if (delta_time > KEY_RELEASE_TIME)
    {
      key_item->state = KEY_RELEASED;
      return;
    }
  }
  
  if ((key_item->state == KEY_PRESSED) && (key_item->current_state != 0))
  {
    //key still presed now
    uint32_t delta_time = signal_capture_get_packet_cnt() - key_item->key_timestamp;
    if (delta_time > KEY_HOLD_TIME)
    {
      key_item->state = KEY_HOLD;
      return;
    }
  }
}
