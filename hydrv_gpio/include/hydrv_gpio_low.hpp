#ifndef HYDRV_GPIO_LOW_H_
#define HYDRV_GPIO_LOW_H_

#include <cstdint>
#include <cstddef>

extern "C"
{
#include "stm32f4xx.h"

#include "hydrolib_common.h"
}

namespace hydrv::GPIO
{
    class GPIOLow
    {
    public:
        struct GPIOGroup
        {
            friend class GPIOLow;

        public:
            static constexpr std::size_t PIN_COUNT = 16;

        public:
            GPIOGroup(GPIO_TypeDef *const GPIOx, const uint32_t RCC_AHB1ENR_GPIOxEN)
                : GPIOx_(GPIOx),
                  RCC_AHB1ENR_GPIOxEN_(RCC_AHB1ENR_GPIOxEN)

            {
            }

        private:
            GPIO_TypeDef *const GPIOx_;
            const uint32_t RCC_AHB1ENR_GPIOxEN_;

            bool inited_pins_[PIN_COUNT] = {0};
        };

    public:
        static GPIOGroup GPIOD_group;
        static GPIOGroup GPIOC_group;

    public:
        GPIOLow(GPIOGroup &GPIO_group, unsigned pin)
            : is_inited_(GPIO_group.inited_pins_[pin]),
              GPIOx_(GPIO_group.GPIOx_),
              pin_(pin),
              output_speed_reg_mask_(0x3UL << (2 * pin)),
              output_speed_reg_value_low_(0x0UL),
              output_speed_reg_value_very_high_(0x3UL << (2 * pin)),
              output_type_reg_mask_(0x1UL << pin),
              push_pull_reg_mask_(0x3UL << (2 * pin)),
              push_pull_reg_value_no_(0x0UL),
              mode_reg_mask_(0x3UL << (2 * pin)),
              mode_reg_value_output_(0x1UL << (2 * pin)),
              mode_reg_value_altfunc_(0x2UL << (2 * pin)),
              altfunc_reg_low_mask_(0xFUL << (4 * pin)),
              altfunc_reg_high_mask_(0xFUL << (4 * (pin - 8))),
              set_reg_mask_(0x1UL << pin),
              reset_reg_mask_(0x1UL << (pin + GPIO_BSRR_BR0_Pos))
        {
            EnableGPIOxClock_(GPIO_group.RCC_AHB1ENR_GPIOxEN_);
        }

    public:
        inline hydrolib_ReturnCode InitAsOutput()
        {
            if (is_inited_)
            {
                return HYDROLIB_RETURN_FAIL;
            }

            SetPinConfig_(output_speed_reg_value_low_, false, push_pull_reg_value_no_);
            SetModeOutput_();

            is_inited_ = true;

            return HYDROLIB_RETURN_OK;
        }

        inline hydrolib_ReturnCode InitAsUART(uint32_t altfunc)
        {
            if (is_inited_)
            {
                return HYDROLIB_RETURN_FAIL;
            }

            SetPinConfig_(output_speed_reg_value_very_high_, false, push_pull_reg_value_no_);
            SetModeAltfunc_(altfunc);

            is_inited_ = true;

            return HYDROLIB_RETURN_OK;
        }

        inline bool IsInited()
        {
            return is_inited_;
        }

        inline void Set()
        {
            CLEAR_BIT(GPIOx_->BSRR, reset_reg_mask_);
            SET_BIT(GPIOx_->BSRR, set_reg_mask_);
        }

        inline void Reset()
        {
            SET_BIT(GPIOx_->BSRR, reset_reg_mask_);
            CLEAR_BIT(GPIOx_->BSRR, set_reg_mask_);
        }

    private:
        bool &is_inited_;
        GPIO_TypeDef *const GPIOx_;
        const unsigned pin_;

        const uint32_t output_speed_reg_mask_;
        const uint32_t output_speed_reg_value_low_;
        const uint32_t output_speed_reg_value_very_high_;

        const uint32_t output_type_reg_mask_;

        const uint32_t push_pull_reg_mask_;
        const uint32_t push_pull_reg_value_no_;

        const uint32_t mode_reg_mask_;
        const uint32_t mode_reg_value_output_;
        const uint32_t mode_reg_value_altfunc_;

        const uint32_t altfunc_reg_low_mask_;
        const uint32_t altfunc_reg_high_mask_;

        const uint32_t set_reg_mask_;
        const uint32_t reset_reg_mask_;

    private:
        static inline void EnableGPIOxClock_(const uint32_t RCC_AHB1ENR_GPIOxEN)
        {
            __IO uint32_t tmpreg = 0x00U;
            SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOxEN); /* Delay after an RCC peripheral clock enabling */
            tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOxEN);
            (void)tmpreg;
        }

        inline void SetPinConfig_(uint32_t speed, bool is_open_drain, uint32_t push_pull)
        {
            MODIFY_REG(GPIOx_->OSPEEDR, output_speed_reg_mask_, speed);
            if (is_open_drain)
            {
                SET_BIT(GPIOx_->OTYPER, output_type_reg_mask_);
            }
            else
            {
                CLEAR_BIT(GPIOx_->OTYPER, output_type_reg_mask_);
            }
            MODIFY_REG(GPIOx_->PUPDR, push_pull_reg_mask_, push_pull);
        }

        inline void SetModeOutput_()
        {
            MODIFY_REG(GPIOx_->MODER, mode_reg_mask_, mode_reg_value_output_);
        }

        inline void SetModeAltfunc_(uint32_t altfunc)
        {
            if (pin_ > 7)
            {
                MODIFY_REG(GPIOx_->AFR[1], altfunc_reg_high_mask_, GetAltfuncRegHighValue_(altfunc));
            }
            else
            {
                MODIFY_REG(GPIOx_->AFR[0], altfunc_reg_low_mask_, GetAltfuncRegLowValue_(altfunc));
            }

            MODIFY_REG(GPIOx_->MODER, mode_reg_mask_, mode_reg_value_altfunc_);
        }

        inline uint32_t GetAltfuncRegLowValue_(uint32_t func)
        {
            return func << (4 * pin_);
        }

        inline uint32_t GetAltfuncRegHighValue_(uint32_t func)
        {
            return func << (4 * (pin_ - 8));
        }
    };
}

#endif
