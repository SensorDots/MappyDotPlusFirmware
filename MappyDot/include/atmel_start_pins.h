/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <port.h>

/**
 * \brief Set ADDR_IN pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void ADDR_IN_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTB_set_pin_pull_mode(6, pull_mode);
}

/**
 * \brief Set ADDR_IN data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void ADDR_IN_set_dir(const enum port_dir dir)
{
    PORTB_set_pin_dir(6, dir);
}

/**
 * \brief Set ADDR_IN level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void ADDR_IN_set_level(const bool level)
{
    PORTB_set_pin_level(6, level);
}

/**
 * \brief Toggle output level on ADDR_IN
 *
 * Toggle the pin level
 */
static inline void ADDR_IN_toggle_level()
{
    PORTB_toggle_pin_level(6);
}

/**
 * \brief Get level on ADDR_IN
 *
 * Reads the level on a pin
 */
static inline bool ADDR_IN_get_level()
{
    return PORTB_get_pin_level(6);
}

/**
 * \brief Set MST pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void MST_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTB_set_pin_pull_mode(7, pull_mode);
}

/**
 * \brief Set MST data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void MST_set_dir(const enum port_dir dir)
{
    PORTB_set_pin_dir(7, dir);
}

/**
 * \brief Set MST level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void MST_set_level(const bool level)
{
    PORTB_set_pin_level(7, level);
}

/**
 * \brief Toggle output level on MST
 *
 * Toggle the pin level
 */
static inline void MST_toggle_level()
{
    PORTB_toggle_pin_level(7);
}

/**
 * \brief Get level on MST
 *
 * Reads the level on a pin
 */
static inline bool MST_get_level()
{
    return PORTB_get_pin_level(7);
}

/**
 * \brief Set SYNC pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void SYNC_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTC_set_pin_pull_mode(0, pull_mode);
}

/**
 * \brief Set SYNC data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void SYNC_set_dir(const enum port_dir dir)
{
    PORTC_set_pin_dir(0, dir);
}

/**
 * \brief Set SYNC level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void SYNC_set_level(const bool level)
{
    PORTC_set_pin_level(0, level);
}

/**
 * \brief Toggle output level on SYNC
 *
 * Toggle the pin level
 */
static inline void SYNC_toggle_level()
{
    PORTC_toggle_pin_level(0);
}

/**
 * \brief Get level on SYNC
 *
 * Reads the level on a pin
 */
static inline bool SYNC_get_level()
{
    return PORTC_get_pin_level(0);
}

/**
 * \brief Set ADDR_OUT pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void ADDR_OUT_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTC_set_pin_pull_mode(1, pull_mode);
}

/**
 * \brief Set ADDR_OUT data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void ADDR_OUT_set_dir(const enum port_dir dir)
{
    PORTC_set_pin_dir(1, dir);
}

/**
 * \brief Set ADDR_OUT level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void ADDR_OUT_set_level(const bool level)
{
    PORTC_set_pin_level(1, level);
}

/**
 * \brief Toggle output level on ADDR_OUT
 *
 * Toggle the pin level
 */
static inline void ADDR_OUT_toggle_level()
{
    PORTC_toggle_pin_level(1);
}

/**
 * \brief Get level on ADDR_OUT
 *
 * Reads the level on a pin
 */
static inline bool ADDR_OUT_get_level()
{
    return PORTC_get_pin_level(1);
}

/**
 * \brief Set XSHUT pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void XSHUT_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTC_set_pin_pull_mode(2, pull_mode);
}

/**
 * \brief Set XSHUT data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void XSHUT_set_dir(const enum port_dir dir)
{
    PORTC_set_pin_dir(2, dir);
}

/**
 * \brief Set XSHUT level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void XSHUT_set_level(const bool level)
{
    PORTC_set_pin_level(2, level);
}

/**
 * \brief Toggle output level on XSHUT
 *
 * Toggle the pin level
 */
static inline void XSHUT_toggle_level()
{
    PORTC_toggle_pin_level(2);
}

/**
 * \brief Get level on XSHUT
 *
 * Reads the level on a pin
 */
static inline bool XSHUT_get_level()
{
    return PORTC_get_pin_level(2);
}

/**
 * \brief Set GPIO1 pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void GPIO1_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTC_set_pin_pull_mode(3, pull_mode);
}

/**
 * \brief Set GPIO1 data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void GPIO1_set_dir(const enum port_dir dir)
{
    PORTC_set_pin_dir(3, dir);
}

/**
 * \brief Set GPIO1 level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void GPIO1_set_level(const bool level)
{
    PORTC_set_pin_level(3, level);
}

/**
 * \brief Toggle output level on GPIO1
 *
 * Toggle the pin level
 */
static inline void GPIO1_toggle_level()
{
    PORTC_toggle_pin_level(3);
}

/**
 * \brief Get level on GPIO1
 *
 * Reads the level on a pin
 */
static inline bool GPIO1_get_level()
{
    return PORTC_get_pin_level(3);
}

/**
 * \brief Set PC4 pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void PC4_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTC_set_pin_pull_mode(4, pull_mode);
}

/**
 * \brief Set PC4 data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void PC4_set_dir(const enum port_dir dir)
{
    PORTC_set_pin_dir(4, dir);
}

/**
 * \brief Set PC4 level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void PC4_set_level(const bool level)
{
    PORTC_set_pin_level(4, level);
}

/**
 * \brief Toggle output level on PC4
 *
 * Toggle the pin level
 */
static inline void PC4_toggle_level()
{
    PORTC_toggle_pin_level(4);
}

/**
 * \brief Get level on PC4
 *
 * Reads the level on a pin
 */
static inline bool PC4_get_level()
{
    return PORTC_get_pin_level(4);
}

/**
 * \brief Set PC5 pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void PC5_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTC_set_pin_pull_mode(5, pull_mode);
}

/**
 * \brief Set PC5 data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void PC5_set_dir(const enum port_dir dir)
{
    PORTC_set_pin_dir(5, dir);
}

/**
 * \brief Set PC5 level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void PC5_set_level(const bool level)
{
    PORTC_set_pin_level(5, level);
}

/**
 * \brief Toggle output level on PC5
 *
 * Toggle the pin level
 */
static inline void PC5_toggle_level()
{
    PORTC_toggle_pin_level(5);
}

/**
 * \brief Get level on PC5
 *
 * Reads the level on a pin
 */
static inline bool PC5_get_level()
{
    return PORTC_get_pin_level(5);
}

/**
 * \brief Set GND pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void GND_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTD_set_pin_pull_mode(3, pull_mode);
}

/**
 * \brief Set GND data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void GND_set_dir(const enum port_dir dir)
{
    PORTD_set_pin_dir(3, dir);
}

/**
 * \brief Set GND level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void GND_set_level(const bool level)
{
    PORTD_set_pin_level(3, level);
}

/**
 * \brief Set all unused pins to pulldown to save power
 *
 */
static inline void UNUSED_set_input()
{
   //PD4,PD2,PD1,PD0,PE2,PD6,PD7,PB0,PB1,PB2
   PORTD_set_pin_dir(4, PORT_DIR_IN);
   PORTD_set_pin_dir(2, PORT_DIR_IN);
   PORTD_set_pin_dir(0, PORT_DIR_IN);
   PORTE_set_pin_dir(2, PORT_DIR_IN);
   PORTD_set_pin_dir(6, PORT_DIR_IN);
   PORTD_set_pin_dir(7, PORT_DIR_IN);
   PORTB_set_pin_dir(0, PORT_DIR_IN);
   PORTB_set_pin_dir(1, PORT_DIR_IN);
   PORTB_set_pin_dir(2, PORT_DIR_IN);
}

/**
 * \brief Toggle output level on GND
 *
 * Toggle the pin level
 */
static inline void GND_toggle_level()
{
    PORTD_toggle_pin_level(3);
}

/**
 * \brief Get level on GND
 *
 * Reads the level on a pin
 */
static inline bool GND_get_level()
{
    return PORTD_get_pin_level(3);
}

/**
 * \brief Set GND2 pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void GND2_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTD_set_pin_pull_mode(5, pull_mode);
}

/**
 * \brief Set GND2 data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void GND2_set_dir(const enum port_dir dir)
{
    PORTD_set_pin_dir(5, dir);
}

/**
 * \brief Set GND2 level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void GND2_set_level(const bool level)
{
    PORTD_set_pin_level(5, level);
}

/**
 * \brief Toggle output level on GND2
 *
 * Toggle the pin level
 */
static inline void GND2_toggle_level()
{
    PORTD_toggle_pin_level(5);
}

/**
 * \brief Get level on GND2
 *
 * Reads the level on a pin
 */
static inline bool GND2_get_level()
{
    return PORTD_get_pin_level(5);
}

/**
 * \brief Set PE0 pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void PE0_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTE_set_pin_pull_mode(0, pull_mode);
}

/**
 * \brief Set PE0 data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void PE0_set_dir(const enum port_dir dir)
{
    PORTE_set_pin_dir(0, dir);
}

/**
 * \brief Set PE0 level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void PE0_set_level(const bool level)
{
    PORTE_set_pin_level(0, level);
}

/**
 * \brief Toggle output level on PE0
 *
 * Toggle the pin level
 */
static inline void PE0_toggle_level()
{
    PORTE_toggle_pin_level(0);
}

/**
 * \brief Get level on PE0
 *
 * Reads the level on a pin
 */
static inline bool PE0_get_level()
{
    return PORTE_get_pin_level(0);
}

/**
 * \brief Set PE1 pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void PE1_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTE_set_pin_pull_mode(1, pull_mode);
}

/**
 * \brief Set PE1 data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void PE1_set_dir(const enum port_dir dir)
{
    PORTE_set_pin_dir(1, dir);
}

/**
 * \brief Set PE1 level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void PE1_set_level(const bool level)
{
    PORTE_set_pin_level(1, level);
}

/**
 * \brief Toggle output level on PE1
 *
 * Toggle the pin level
 */
static inline void PE1_toggle_level()
{
    PORTE_toggle_pin_level(1);
}

/**
 * \brief Get level on PE1
 *
 * Reads the level on a pin
 */
static inline bool PE1_get_level()
{
    return PORTE_get_pin_level(1);
}

/**
 * \brief Set LED pull mode
 *
 * Configure pin to pull up, down or disable pull mode, supported pull
 * modes are defined by device used
 *
 * \param[in] pull_mode Pin pull mode
 */
static inline void LED_set_pull_mode(const enum port_pull_mode pull_mode)
{
    PORTE_set_pin_pull_mode(3, pull_mode);
}

/**
 * \brief Set LED data direction
 *
 * Select if the pin data direction is input, output or disabled.
 * If disabled state is not possible, this function throws an assert.
 *
 * \param[in] direction PORT_DIR_IN  = Data direction in
 *                      PORT_DIR_OUT = Data direction out
 *                      PORT_DIR_OFF = Disables the pin
 *                      (low power state)
 */
static inline void LED_set_dir(const enum port_dir dir)
{
    PORTE_set_pin_dir(3, dir);
}

/**
 * \brief Set LED level
 *
 * Sets output level on a pin
 *
 * \param[in] level true  = Pin level set to "high" state
 *                  false = Pin level set to "low" state
 */
static inline void LED_set_level(const bool level)
{
    PORTE_set_pin_level(3, level);
}

/**
 * \brief Toggle output level on LED
 *
 * Toggle the pin level
 */
static inline void LED_toggle_level()
{
    PORTE_toggle_pin_level(3);
}

/**
 * \brief Get level on LED
 *
 * Reads the level on a pin
 */
static inline bool LED_get_level()
{
    return PORTE_get_pin_level(3);
}

#endif /* ATMEL_START_PINS_H_INCLUDED */
