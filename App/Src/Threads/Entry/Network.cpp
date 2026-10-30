#include "Threads/Entry/Network.hpp"
#include "Data/Writers.hpp"
#include "Utilities/config.h"
#include "Utilities/utilities_buzzer.h"
#include "Context.hpp"
#include "nx_stm32_phy_driver.h"
#include "main.h"
#include "stm32h7xx_hal_gpio.h"
#include "tx_api.h"

namespace Thread
{
    namespace Entry
    {
        int idle_beep_counter {};
        void Network(unsigned long thread_input)
        {
            auto writer = reinterpret_cast<Telemetry::Writers::Networking*>(thread_input);
            writer->network_thread_online.store(true);
            while (true)
            {
                Network::manager->poll();

                const bool link_up = nx_eth_phy_get_link_state() > ETH_PHY_STATUS_LINK_DOWN;
                HAL_GPIO_WritePin(LED_ETH_GPIO_Port, LED_ETH_Pin, link_up ? GPIO_PIN_SET : GPIO_PIN_RESET);

                Network::broadcaster->dispatch(RTOS_TICKS_TO_MS(tx_time_get()));
                Network::commands->poll();
                if (System::current_state.update_and_read() == STATE_NAME::IDLE) {
                    if (++idle_beep_counter >= 500) {
                        HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
                        play_note(&htim15, TIM_CHANNEL_1, 200, MUSIC_A3, 100);
                        HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
                        idle_beep_counter = 0;
                    }
                } else {
                    idle_beep_counter = 0;
                }

                tx_thread_sleep(NETWORK_THREAD_SLEEP_TIMER);
            }

            writer->network_thread_online.store(false);
        }
    }
}
