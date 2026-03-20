// FreeRTOS相关函数实现文件
#include "user_freertos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "USART1.h"

/*任务句柄*/
TaskHandle_t start_task_handle;
TaskHandle_t task_2ms_handle;
TaskHandle_t task_4ms_handle;
TaskHandle_t task_6ms_handle;
TaskHandle_t task_10ms_handle;
TaskHandle_t task_20ms_handle;
TaskHandle_t task_50ms_handle;
TaskHandle_t task_500ms_handle;
TaskHandle_t task_1000ms_handle;

/*任务声明*/
void start_task(void *pvParameters);
void task_2ms(void *pvParameters);
void task_4ms(void *pvParameters);
void task_6ms(void *pvParameters);
void task_10ms(void *pvParameters);
void task_20ms(void *pvParameters);
void task_50ms(void *pvParameters);
void task_500ms(void *pvParameters);
void task_1000ms(void *pvParameters);

void user_freertos_start(void)
{
    /*创建开始任务*/
    BaseType_t ret = xTaskCreate((TaskFunction_t)start_task,
                (const char *)"start_task", 
                (configSTACK_DEPTH_TYPE)START_TASK_STACK_SIZE,
                (void *)NULL,
                (UBaseType_t)START_TASK_PRIORITY,
                (TaskHandle_t *)&start_task_handle);

    if(ret != pdPASS)
    {
        printf("Start task create failed!\r\n"); // 增加创建失败检测
        return;
    }

    /*开启任务调度*/
    vTaskStartScheduler();
}

void start_task(void *pvParameters)
{
    /*进入临界区（创建任务时关闭中断，避免干扰）*/
    taskENTER_CRITICAL();

    // 2ms核心任务（绝对精准周期）
    if(xTaskCreate((TaskFunction_t)task_2ms,
            (const char *)"task_2ms", 
            (configSTACK_DEPTH_TYPE)TASK_2MS_STACK_SIZE,
            (void *)NULL,
            (UBaseType_t)TASK_2MS_PRIORITY,
            (TaskHandle_t *)&task_2ms_handle) == pdPASS)
        printf("task_2ms created successfully!\r\n");
    else
        printf("task_2ms create failed!\r\n");

    // 4ms任务
    if(xTaskCreate((TaskFunction_t)task_4ms,
            (const char *)"task_4ms", 
            (configSTACK_DEPTH_TYPE)TASK_4MS_STACK_SIZE,
            (void *)NULL,
            (UBaseType_t)TASK_4MS_PRIORITY,
            (TaskHandle_t *)&task_4ms_handle) == pdPASS)
        printf("task_4ms created successfully!\r\n");
    else
        printf("task_4ms create failed!\r\n");

    // 6ms任务
    if(xTaskCreate((TaskFunction_t)task_6ms,
            (const char *)"task_6ms", 
            (configSTACK_DEPTH_TYPE)TASK_6MS_STACK_SIZE,
            (void *)NULL,
            (UBaseType_t)TASK_6MS_PRIORITY,
            (TaskHandle_t *)&task_6ms_handle) == pdPASS)
        printf("task_6ms created successfully!\r\n");
    else
        printf("task_6ms create failed!\r\n");

    // 10ms任务
    if(xTaskCreate((TaskFunction_t)task_10ms,
            (const char *)"task_10ms", 
            (configSTACK_DEPTH_TYPE)TASK_10MS_STACK_SIZE,
            (void *)NULL,
            (UBaseType_t)TASK_10MS_PRIORITY,
            (TaskHandle_t *)&task_10ms_handle) == pdPASS)
        printf("task_10ms created successfully!\r\n");
    else
        printf("task_10ms create failed!\r\n");

    // 20ms任务
    if(xTaskCreate((TaskFunction_t)task_20ms,
            (const char *)"task_20ms", 
            (configSTACK_DEPTH_TYPE)TASK_20MS_STACK_SIZE,
            (void *)NULL,
            (UBaseType_t)TASK_20MS_PRIORITY,
            (TaskHandle_t *)&task_20ms_handle) == pdPASS)
        printf("task_20ms created successfully!\r\n");
    else
        printf("task_20ms create failed!\r\n");

    // 50ms任务
    if(xTaskCreate((TaskFunction_t)task_50ms,
            (const char *)"task_50ms", 
            (configSTACK_DEPTH_TYPE)TASK_50MS_STACK_SIZE,
            (void *)NULL,
            (UBaseType_t)TASK_50MS_PRIORITY,
            (TaskHandle_t *)&task_50ms_handle) == pdPASS)
        printf("task_50ms created successfully!\r\n");
    else
        printf("task_50ms create failed!\r\n");
    
    // 500ms任务（喂狗）
    if(xTaskCreate((TaskFunction_t)task_500ms,
            (const char *)"task_500ms", 
            (configSTACK_DEPTH_TYPE)TASK_500MS_STACK_SIZE,
            (void *)NULL,
            (UBaseType_t)TASK_500MS_PRIORITY,
            (TaskHandle_t *)&task_500ms_handle) == pdPASS)
        printf("task_500ms created successfully!\r\n");
    else
        printf("task_500ms create failed!\r\n");

    // 1000ms任务
    if(xTaskCreate((TaskFunction_t)task_1000ms,
            (const char *)"task_1000ms", 
            (configSTACK_DEPTH_TYPE)TASK_1000MS_STACK_SIZE,
            (void *)NULL,
            (UBaseType_t)TASK_1000MS_PRIORITY,
            (TaskHandle_t *)&task_1000ms_handle) == pdPASS)
        printf("task_1000ms created successfully!\r\n");
    else
        printf("task_1000ms create failed!\r\n");

    // 删除启动任务（完成使命）
    vTaskDelete(start_task_handle);
    /*退出临界区*/
    taskEXIT_CRITICAL();
}

// 2ms核心任务（用vTaskDelayUntil实现绝对精准周期）
void task_2ms(void *pvParameters)
{
    TickType_t xLastWakeTime;
    // 初始化上次唤醒时间为当前系统时间
    xLastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        // ********** 核心任务执行区 **********
        taskENTER_CRITICAL(); // 临界区保护全局变量
        Duty_2ms();        
        taskEXIT_CRITICAL();
        // ********** 精准延时区 **********
        // 绝对延时：保证任务每2ms执行一次，不受执行耗时影响
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
    }
}

// 4ms任务（精准周期）
void task_4ms(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        taskENTER_CRITICAL();
        Duty_4ms();        
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
    }
}

// 6ms任务（精准周期）
void task_6ms(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        taskENTER_CRITICAL();
        Duty_6ms();        
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(6));
    }
}

// 10ms任务（精准周期）
void task_10ms(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        taskENTER_CRITICAL();
        Duty_10ms();       
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

// 20ms任务（精准周期）
void task_20ms(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        taskENTER_CRITICAL();
        Duty_20ms();       
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

// 50ms任务（精准周期）
void task_50ms(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        taskENTER_CRITICAL();
        Duty_50ms();       
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

// 500ms任务（喂狗）
void task_500ms(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        taskENTER_CRITICAL();
        Duty_500ms();       
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}

// 1000ms任务（状态统计）
void task_1000ms(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        taskENTER_CRITICAL();
        Duty_1000ms();     
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}