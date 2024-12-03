/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Real-Time Clock
*              basics example of ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "string.h"
#include "time.h"
#include <inttypes.h>

/*******************************************************************************
* Macros
*******************************************************************************/
#define UART_TIMEOUT_MS (10u)      /* in milliseconds */
#define INPUT_TIMEOUT_MS (120000u) /* in milliseconds */

#define STRING_BUFFER_SIZE (80)

/* Available commands */
#define RTC_CMD_SET_DATE_TIME ('1')
#define RTC_CMD_CONFIG_DST ('2')

#define RTC_CMD_ENABLE_DST ('1')
#define RTC_CMD_DISABLE_DST ('2')
#define RTC_CMD_QUIT_CONFIG_DST ('3')

#define FIXED_DST_FORMAT ('1')
#define RELATIVE_DST_FORMAT ('2')

/* Macro used for checking validity of user input */
#define MIN_SPACE_KEY_COUNT_NEW_TIME (5)
#define MIN_SPACE_KEY_COUNT_DST_TIME (3)

/* Structure tm stores years since 1900 */
#define TM_YEAR_BASE (1900u)

/* Maximum value of seconds and minutes */
#define MAX_SEC_OR_MIN (60u)

/* Maximum value of hours definition */
#define MAX_HOURS_24H (23UL)

/* Month per year definition */
#define MONTHS_PER_YEAR (12U)

/* Days per week definition */
#define DAYS_PER_WEEK (7u)

/* Days in month */
#define DAYS_IN_JANUARY (31U)   /* Number of days in January */
#define DAYS_IN_FEBRUARY (28U)  /* Number of days in February */
#define DAYS_IN_MARCH (31U)     /* Number of days in March */
#define DAYS_IN_APRIL (30U)     /* Number of days in April */
#define DAYS_IN_MAY (31U)       /* Number of days in May */
#define DAYS_IN_JUNE (30U)      /* Number of days in June */
#define DAYS_IN_JULY (31U)      /* Number of days in July */
#define DAYS_IN_AUGUST (31U)    /* Number of days in August */
#define DAYS_IN_SEPTEMBER (30U) /* Number of days in September */
#define DAYS_IN_OCTOBER (31U)   /* Number of days in October */
#define DAYS_IN_NOVEMBER (30U)  /* Number of days in November */
#define DAYS_IN_DECEMBER (31U)  /* Number of days in December */

/* Flags to indicate the if the entered time is valid */
#define DST_DISABLED_FLAG (0)
#define DST_VALID_START_TIME_FLAG (1)
#define DST_VALID_END_TIME_FLAG (2)
#define DST_ENABLED_FLAG (3)

/* Macro to validate seconds parameter */
#define IS_SEC_VALID(sec) ((sec) <= MAX_SEC_OR_MIN)

/* Macro to validate minutes parameters */
#define IS_MIN_VALID(min) ((min) <= MAX_SEC_OR_MIN)

/* Macro to validate hour parameter */
#define IS_HOUR_VALID(hour) ((hour) <= MAX_HOURS_24H)

/* Macro to validate month parameter */
#define IS_MONTH_VALID(month) (((month) > 0U) && ((month) <= MONTHS_PER_YEAR))

/* Macro to validate the year value */
#define IS_YEAR_VALID(year) ((year) > 0U)

/* Checks whether the year passed through the parameter is leap or not */
#define IS_LEAP_YEAR(year) \
(((0U == (year % 4UL)) && (0U != (year % 100UL))) || (0U == (year % 400UL)))

/*******************************************************************************
* Global Variables
*******************************************************************************/
static uint32_t century_data = 2000;
static cy_stc_rtc_config_t current_time;
/* Variables used to store DST start and end time information */
static cy_stc_rtc_dst_t dst_time;
const cy_stc_sysint_t IRQ_CFG_RTC_ALARM2 =
{
    .intrSrc = ((NvicMux3_IRQn << 16) | srss_interrupt_backup_IRQn),
    .intrPriority = 0u,
};

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void handle_error(void);
static void rtc_isr(void);
static void construct_time_format(struct tm *time);
static void set_new_time(uint32_t timeout_ms);
static bool validate_date_time(uint32_t sec, uint32_t min, uint32_t hour,
                               uint32_t mday, uint32_t month, uint32_t year);
static void set_dst_feature(uint32_t timeout_ms);
static cy_rslt_t fetch_time_data(char *buffer,
                                 uint32_t timeout_ms,
                                 uint32_t *space_count);
static uint32_t get_week_of_month(uint32_t day, uint32_t month, uint32_t year);
static cy_en_scb_uart_status_t get_character(CySCB_Type * base,
                                             uint8_t *value,
                                             uint32_t timeout);

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*   This function:
*  - Initializes the device and board peripherals
*  - Initializes RTC
*  - The loop checks for the user command and process the commands
*
* Parameters :
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t rslt;
    uint8_t cmd;
    char buffer[STRING_BUFFER_SIZE];
    struct tm date_time;

    /* Initialize the device and board peripherals */
    rslt = cybsp_init();
    if (CY_RSLT_SUCCESS != rslt)
    {
        handle_error();
    }

    /* Initialize retarget-io to use the debug UART port */
    Cy_SCB_UART_Init(UART_HW, &UART_config, NULL);
    Cy_SCB_UART_Enable(UART_HW);
    rslt = cy_retarget_io_init(UART_HW);

    /* retarget-io init failed. Stop program execution */
    if (CY_RSLT_SUCCESS != rslt)
    {
        CY_ASSERT(0);
    }

    printf("retarget-io ver1.6 testing \r\n");

    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("****************** PDL: RTC Basics ******************\r\n\n");

    /* Set RTC clock source */
    Cy_RTC_SelectClockSource(CY_RTC_CLK_SELECT_ILO);

    /* If the Power-on reset occurs, it initializes RTC */
    if (CY_SYSLIB_RESET_PORVDDD ==
       (CY_SYSLIB_RESET_PORVDDD & Cy_SysLib_GetResetReason()))
    {
        rslt = Cy_RTC_Init(&RTC_config);
    }
    /* If it is a first execution, it initializes RTC */
    else if (!Cy_RTC_IsExternalResetOccurred())
    {
        rslt = Cy_RTC_Init(&RTC_config);
    }

    if (CY_RTC_SUCCESS != rslt)
    {
        handle_error();
    }

    /* Clear reset reason */
    Cy_SysLib_ClearResetReason();

    /* Set interrupt service routine */
    Cy_SysInt_Init(&IRQ_CFG_RTC_ALARM2, &rtc_isr);

    IRQn_Type irqn = Cy_SysInt_GetNvicConnection(srss_interrupt_backup_IRQn);
    NVIC_EnableIRQ(irqn);

    /* Display available commands */
    printf("Available commands \r\n");
    printf("1 : Set new time and date\r\n");
    printf("2 : Configure DST feature\r\n\n");


    for (;;)
    {
        uint32_t savedIntrStatus = Cy_SysLib_EnterCriticalSection();

        /* Get current time */
        Cy_RTC_GetDateAndTime(&current_time);

        Cy_SysLib_ExitCriticalSection(savedIntrStatus);

        /* Construct cy_stc_rtc_config_t to struct tm format */
        construct_time_format(&date_time);

        /* Print current time */
        strftime(buffer, sizeof(buffer), "%c", &date_time);
        printf("\r%s", buffer);
        memset(buffer, '\0', sizeof(buffer));

        /* Check if any command is input */
        rslt = get_character(UART_HW, &cmd, UART_TIMEOUT_MS);
        if (CY_SCB_UART_BAD_PARAM != rslt)
        {
            if (RTC_CMD_SET_DATE_TIME == cmd)
            {
                printf("\r[Command] : Set new time\r\n");
                set_new_time(INPUT_TIMEOUT_MS);
            }
            else if (RTC_CMD_CONFIG_DST == cmd)
            {
                printf("\r[Command] : Configure DST feature\r\n");
                set_dst_feature(INPUT_TIMEOUT_MS);
            }
        }
    }
}

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
*  User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: rtc_isr
********************************************************************************
* Summary:
*  RTC interrupt service routine to handle interrupt sources.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void rtc_isr(void)
{
    Cy_RTC_Interrupt(&dst_time, true);
}

/*******************************************************************************
* Function Name: construct_time_format
********************************************************************************
* Summary:
*  This functions constructs cy_stc_rtc_config_t to struct tm format.
*
* Parameter:
*  struct tm *time : time with struct tm format
*
* Return:
*  void
*******************************************************************************/
static void construct_time_format(struct tm *time)
{
    static const uint16_t CUMULATIVE_DAYS[] = {0, 31, 59, 90, 120, 151,
                                               181, 212, 243, 273, 304, 334};
    time->tm_sec = current_time.sec;
    time->tm_min = current_time.min;
    time->tm_hour = current_time.hour;
    time->tm_mday = current_time.date;
    time->tm_mon = (current_time.month - 1u);
    time->tm_year = (current_time.year + century_data - TM_YEAR_BASE);
    time->tm_wday = (current_time.dayOfWeek - 1u);
    time->tm_yday = CUMULATIVE_DAYS[time->tm_mon] + (current_time.date - 1) +
        (((current_time.month) >= 3 &&
         (Cy_RTC_IsLeapYear(current_time.year + century_data) ? 1u : 0u)));
    time->tm_isdst = -1;
}

/*******************************************************************************
* Function Name: set_dst_feature
********************************************************************************
* Summary:
*  This functions takes the user input ,sets the dst start/end date and time,
*  and then enables the DST feature.
*
* Parameter:
*  uint32_t timeout_ms : Maximum allowed time (in milliseconds) for the
*  function
*
* Return:
*  void
*******************************************************************************/
static void set_dst_feature(uint32_t timeout_ms)
{
    cy_rslt_t rslt;
    uint8_t dst_cmd;
    char dst_start_buffer[STRING_BUFFER_SIZE] = {0};
    char dst_end_buffer[STRING_BUFFER_SIZE] = {0};
    uint32_t space_count = 0;
    static uint32_t dst_data_flag = 0;

    /* Variables used to store date and time information */
    uint32_t mday = 0, month = 0, year = 0, sec = 0, min = 0, hour = 0;
    uint8_t fmt = 0;
    if (DST_ENABLED_FLAG == dst_data_flag)
    {
        Cy_RTC_GetDateAndTime(&current_time);

        if (Cy_RTC_GetDstStatus(&dst_time, &current_time))
        {
            printf("\rCurrent DST Status :: Active\r\n\n");
        }
        else
        {
            printf("\rCurrent DST Status :: Inactive\r\n\n");
        }
    }
    else
    {
        printf("\rCurrent DST Status :: Disabled\r\n\n");
    }

    /* Display available commands */
    printf("Available DST commands \r\n");
    printf("1 : Enable DST feature\r\n");
    printf("2 : Disable DST feature\r\n");
    printf("3 : Quit DST Configuration\r\n\n");

    /* Get user input via UART */
    rslt = get_character(UART_HW, &dst_cmd, timeout_ms);

    if (rslt != CY_SCB_UART_BAD_PARAM)
    {
        if (RTC_CMD_ENABLE_DST == dst_cmd)
        {
            /* Get DST start time information */
            printf("Enter DST format \r\n");
            printf("1 : Fixed DST format\r\n");
            printf("2 : Relative DST format\r\n\n");

            /* Get user input via UART */
            rslt = get_character(UART_HW, &fmt, timeout_ms);
            if (rslt != CY_SCB_UART_BAD_PARAM)
            {
                printf("Enter DST start time in \"HH dd mm yyyy\" format\r\n");
                rslt = fetch_time_data(dst_start_buffer, timeout_ms,
                                                        &space_count);
                if (rslt != CY_SCB_UART_BAD_PARAM)
                {
                    if (space_count != MIN_SPACE_KEY_COUNT_DST_TIME)
                    {
                        printf("\rInvalid values! Please enter "
                               "the values in specified format\r\n");
                    }
                    else
                    {
                        sscanf(dst_start_buffer, "%" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 "",
                               &hour, &mday, &month, &year);

                        if ((validate_date_time(sec, min, hour,
                                                mday, month, year))&&
                                               ((fmt == FIXED_DST_FORMAT) ||
                                                (fmt == RELATIVE_DST_FORMAT)))
                        {
                            dst_time.startDst.format =
                            (fmt == FIXED_DST_FORMAT) ? CY_RTC_DST_FIXED :
                                                        CY_RTC_DST_RELATIVE;
                            dst_time.startDst.hour = hour;
                            dst_time.startDst.month = month;
                            dst_time.startDst.dayOfWeek =
                            (fmt == FIXED_DST_FORMAT) ?
                            1 : Cy_RTC_ConvertDayOfWeek(mday, month, year);
                            dst_time.startDst.dayOfMonth =
                            (fmt == FIXED_DST_FORMAT) ? mday : 1;
                            dst_time.startDst.weekOfMonth =
                            (fmt == FIXED_DST_FORMAT) ?
                            1 : get_week_of_month(mday, month, year);
                            /* Update flag value to indicate that a valid
                               DST start time information has been received*/
                            dst_data_flag = DST_VALID_START_TIME_FLAG;
                        }
                        else
                        {
                            printf("\rInvalid values! Please enter "
                                   "the values in specified format\r\n");
                        }
                    }
                }
                else
                {
                    printf("\rTimeout \r\n");
                }

                if (DST_VALID_START_TIME_FLAG == dst_data_flag)
                {
                    /* Get DST end time information,
                    iff a valid DST start time information is received */
                    printf("Enter DST end time "
                    " in \"HH dd mm yyyy\" format\r\n");
                    rslt = fetch_time_data(dst_end_buffer, timeout_ms,
                                            &space_count);
                    if (rslt != CY_SCB_UART_BAD_PARAM)
                    {
                        if (space_count != MIN_SPACE_KEY_COUNT_DST_TIME)
                        {
                            printf("\rInvalid values! Please"
                            "enter the values in specified format\r\n");
                        }
                        else
                        {
                            sscanf(dst_end_buffer, "%" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 "",
                                   &hour, &mday, &month, &year);

                            if ((validate_date_time(sec, min, hour,
                                                    mday, month, year))&&
                                                ((fmt == FIXED_DST_FORMAT) ||
                                                 (fmt == RELATIVE_DST_FORMAT)))
                            {
                                dst_time.stopDst.format =
                                      (fmt == FIXED_DST_FORMAT)?
                                       CY_RTC_DST_FIXED : CY_RTC_DST_RELATIVE;
                                dst_time.stopDst.hour = hour;
                                dst_time.stopDst.month = month;
                                dst_time.stopDst.dayOfWeek =
                                (fmt == FIXED_DST_FORMAT) ?
                                1 : Cy_RTC_ConvertDayOfWeek(mday, month, year);
                                dst_time.stopDst.dayOfMonth =
                                (fmt == FIXED_DST_FORMAT) ? mday : 1;
                                dst_time.stopDst.weekOfMonth =
                                (fmt == FIXED_DST_FORMAT) ?
                                1 : get_week_of_month(mday, month, year);
                                /* Update flag value to indicate that a valid
                                 DST end time information has been recieved*/
                                dst_data_flag = DST_VALID_END_TIME_FLAG;
                            }
                            else
                            {
                                printf("\rInvalid values! Please enter the "
                                       " values in specified format\r\n");
                            }
                        }
                    }
                    else
                    {
                        printf("\rTimeout \r\n");
                    }
                }

                if (DST_VALID_END_TIME_FLAG == dst_data_flag)
                {
                    /* set new DST time */
                    Cy_RTC_GetDateAndTime(&current_time);
                    rslt = Cy_RTC_EnableDstTime(&dst_time, &current_time);

                    if (CY_RSLT_SUCCESS == rslt)
                    {
                        dst_data_flag = DST_ENABLED_FLAG;
                        printf("\rDST time updated\r\n\n");
                    }
                    else
                    {
                        handle_error();
                    }
                }
            }
            else
            {
                printf("\rTimeout \r\n");
            }
        }
        else if (RTC_CMD_DISABLE_DST == dst_cmd)
        {
            /* reset dst_time */
            dst_time.stopDst.format = CY_RTC_DST_FIXED;
            dst_time.stopDst.hour = 0;
            dst_time.stopDst.month = 1;
            dst_time.stopDst.dayOfWeek = 1;
            dst_time.stopDst.dayOfMonth = 1;
            dst_time.stopDst.weekOfMonth = 1;
            dst_time.startDst = dst_time.stopDst;

            /* set DST-disabled time */
            Cy_RTC_GetDateAndTime(&current_time);
            rslt = Cy_RTC_EnableDstTime(&dst_time, &current_time);

            if (CY_RSLT_SUCCESS == rslt)
            {
                dst_data_flag = DST_DISABLED_FLAG;
                printf("\rDST feature disabled\r\n\n");
            }
            else
            {
                handle_error();
            }
        }
        else if (RTC_CMD_QUIT_CONFIG_DST == dst_cmd)
        {
            printf("\rExit from DST Configuration \r\n\n");
        }
    }
    else
    {
        printf("\rTimeout \r\n");
    }
}

/*******************************************************************************
* Function Name: set_new_time
********************************************************************************
* Summary:
*  This functions takes the user input and sets the new date and time.
*
* Parameter:
*  uint32_t timeout_ms : Maximum allowed time (in milliseconds) for the
*  function
*
* Return :
*  void
*******************************************************************************/
static void set_new_time(uint32_t timeout_ms)
{
    cy_rslt_t rslt;
    char buffer[STRING_BUFFER_SIZE] = {0};
    uint32_t space_count = 0;

    /* Variables used to store date and time information */
    uint32_t mday, month, year, sec, min, hour;

    printf("\rEnter time in \"HH MM SS dd mm yyyy\" format \r\n");
    rslt = fetch_time_data(buffer, timeout_ms, &space_count);
    if (rslt != CY_SCB_UART_BAD_PARAM)
    {
        if (space_count != MIN_SPACE_KEY_COUNT_NEW_TIME)
        {
            printf("\rInvalid values! Please enter the"
                    "values in specified format\r\n");
        }
        else
        {
            sscanf(buffer, "%" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 "",
                   &hour, &min, &sec,
                   &mday, &month, &year);

            if (validate_date_time(sec, min, hour, mday, month, year))
            {
                rslt = Cy_RTC_SetDateAndTimeDirect(sec, min, hour,
                                                   mday, month, year % 100);

                century_data = ((year / 100) * 100);

                if (CY_RTC_SUCCESS == rslt)
                {
                    printf("\rRTC time updated\r\n\n");
                }
            }
            else
            {
                printf("\rInvalid values! Please enter the values in specified"
                       " format\r\n");
            }
        }
    }
    else
    {
        printf("\rTimeout \r\n");
    }
}

/*******************************************************************************
* Function Name: fetch_time_data
********************************************************************************
* Summary:
*  Function fetches data entered by the user through UART and stores it in the
*  buffer which is passed through parameters. The function also counts number of
*  spaces in the recieved data and stores in the variable, whose address are
*  passsed as parameter.
*
* Parameter:
*  char* buffer        : Buffer to store the fetched data
*  uint32_t timeout_ms : Maximum allowed time (in milliseconds) for the function
*  uint32_t* space_count : The number of spaces present in the fetched data.
*
* Return:
*  Returns the status of the getc request
*
*******************************************************************************/
static cy_rslt_t fetch_time_data(char *buffer, uint32_t timeout_ms,
                                    uint32_t *space_count)
{
    cy_rslt_t rslt;
    uint32_t index = 0;
    uint8_t ch;
    *space_count = 0;
    while (index < STRING_BUFFER_SIZE)
    {
        if (timeout_ms <= UART_TIMEOUT_MS)
        {
            rslt = CY_SCB_UART_BAD_PARAM;
            break;
        }

        rslt = get_character(UART_HW, &ch, UART_TIMEOUT_MS);

        if (rslt != CY_SCB_UART_BAD_PARAM)
        {
            if (ch == '\n' || ch == '\r')
            {
                break;
            }
            else if (ch == ' ')
            {
                (*space_count)++;
            }

            buffer[index] = ch;

            uint32_t count = 0;
            while (count == 0)
            {
                count = Cy_SCB_UART_Put(UART_HW, ch);
            }

            rslt = CY_SCB_UART_SUCCESS;

            index++;
        }

        timeout_ms -= UART_TIMEOUT_MS;
    }
    printf("\n\r");
    return rslt;
}

/*******************************************************************************
* Function Name: get_week_of_month
********************************************************************************
* Summary:
*  Returns week number of the month for a year, month, and day that are passed
*  through parameters.
*
* Parameters:
*  uint32_t day    : The day of the month. Valid range 1..31.
*  uint32_t month  : The month of the year. Valid range 1..12.
*  uint32_t year   : The year value. Valid range non-zero value.
*
* Return:
*  Returns the week number of the month (1 to 5).
*
*******************************************************************************/
static uint32_t get_week_of_month(uint32_t day, uint32_t month, uint32_t year)
{
    uint32_t count = 1;
    uint32_t day_of_week = Cy_RTC_ConvertDayOfWeek(1, month, year);
    uint32_t weekend_day = 8 - day_of_week;
    while (day > weekend_day)
    {
        count++;
        weekend_day += 7;
    }

    return count;
}

/*******************************************************************************
* Function Name: validate_date_time
********************************************************************************
* Summary:
*  This function validates date and time value.
*
* Parameters:
*  uint32_t sec     : The second valid range is [0-59].
*  uint32_t min     : The minute valid range is [0-59].
*  uint32_t hour    : The hour valid range is [0-23].
*  uint32_t date    : The date valid range is [1-31], if the month of February
*                     is selected as the Month parameter, then the valid range
*                     is [0-29].
*  uint32_t month   : The month valid range is [1-12].
*  uint32_t year    : The year valid range is [> 0].
*
* Return:
*  false - invalid ; true - valid
*
*******************************************************************************/
static bool validate_date_time(uint32_t sec, uint32_t min, uint32_t hour,
                               uint32_t mday, uint32_t month, uint32_t year)
{
    static const uint8_t days_in_month_table[MONTHS_PER_YEAR] =
        {
            DAYS_IN_JANUARY,
            DAYS_IN_FEBRUARY,
            DAYS_IN_MARCH,
            DAYS_IN_APRIL,
            DAYS_IN_MAY,
            DAYS_IN_JUNE,
            DAYS_IN_JULY,
            DAYS_IN_AUGUST,
            DAYS_IN_SEPTEMBER,
            DAYS_IN_OCTOBER,
            DAYS_IN_NOVEMBER,
            DAYS_IN_DECEMBER,
        };

    uint8_t days_in_month;

    bool rslt = IS_SEC_VALID(sec) & IS_MIN_VALID(min) &
                IS_HOUR_VALID(hour) & IS_MONTH_VALID(month) &
                IS_YEAR_VALID(year);

    if (rslt)
    {
        days_in_month = days_in_month_table[month - 1];

        if (IS_LEAP_YEAR(year) && (month == 2))
        {
            days_in_month++;
        }

        rslt &= (mday > 0U) && (mday <= days_in_month);
    }

    return rslt;
}

/*******************************************************************************
* Function Name: get_character
********************************************************************************
* Summary: This function gets character via UART.
*
* Parameters:
*  CySCB_Type * base : The pointer to the UART SCB instance.
*  uint8_t *value    : The pointer to store the read value.
*  uint32_t timeout  : It defines when timeout is.
*
* Return
*  cy_en_scb_uart_status_t
*
*******************************************************************************/
static cy_en_scb_uart_status_t get_character(CySCB_Type * base,
                                             uint8_t *value,
                                             uint32_t timeout)
{
    /* Get character via UART */
    uint32_t read_value = Cy_SCB_UART_Get(base);
    uint32_t timeoutTicks = timeout;
    while (read_value == CY_SCB_UART_RX_NO_DATA)
    {
        if (timeout != 0UL)
        {
            if (timeoutTicks > 0UL)
            {
                /* Wait 1ms */
                Cy_SysLib_Delay(1);
                timeoutTicks--;
            }
            else
            {
                return CY_SCB_UART_BAD_PARAM;
            }
        }
        /* Try to get character again */
        read_value = Cy_SCB_UART_Get(base);
    }
    *value = (uint8_t)read_value;
    return CY_SCB_UART_SUCCESS;
}

/* [] END OF FILE */
