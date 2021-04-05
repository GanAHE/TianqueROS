/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

/**
 * OS detection
 */
#ifndef UAVCAN_STM32_CHIBIOS
# define UAVCAN_STM32_CHIBIOS 0
#endif

#ifndef UAVCAN_STM32_NUTTX
# define UAVCAN_STM32_NUTTX 0
#endif

#ifndef UAVCAN_STM32_BAREMETAL
# define UAVCAN_STM32_BAREMETAL 0
#endif

#ifndef UAVCAN_STM32_FREERTOS
# define UAVCAN_STM32_FREERTOS 0
#endif

/**
 * Number of interfaces must be enabled explicitly
 */
#if !defined(UAVCAN_STM32_NUM_IFACES) || (UAVCAN_STM32_NUM_IFACES != 1 && UAVCAN_STM32_NUM_IFACES != 2)
# error "UAVCAN_STM32_NUM_IFACES must be set to either 1 or 2"
#endif

/**
 * Any General-Purpose timer (TIM2, TIM3, TIM4, TIM5)
 * e.g. -DUAVCAN_STM32_TIMER_NUMBER=2
 */
#ifndef UAVCAN_STM32_TIMER_NUMBER
// In this case the clock driver should be implemented by the application
# define UAVCAN_STM32_TIMER_NUMBER 0
#endif
