#include "app_main.h"
#include "bc7215_ac_lib.h"        // 包含 AC 控制库头文件
#include "uart_ring.h"            // 之前定义的控制台环形缓冲
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// --- 全局变量 ---

appConfig_t			appConfig;
bool				acIsFahrenheit = false;

// 状态定义
typedef enum
{
	CHOOSE_UNIT,		// 选择温度制式
    MAIN_MENU,          // 显示主菜单
    CAPTURE,            // 信号采集（配对）
    AC_CONTROL,         // 空调控制
    BACKUP,             // 初始化（配对）信息保存
    RESTORE,            // 使用保存的信息完成初始化（配对）
    FIND_NEXT,          // 尝试下一匹配
    LOAD_PREDEF,        // 加载预定义协议
    IR_PARSING          // 红外指令解析
} AppState_t;
typedef enum
{
    STEP1,
    STEP2,
    STEP3,
    STEP4,
    STEP5,
    STEP6,
    STEP7
} SubState_t;

AppState_t mainState = CHOOSE_UNIT;
SubState_t l2State = STEP1;
SubState_t goBackState;

// 数据缓冲
char inputLine[16];

// AC 库相关变量
bc7215DataMaxPkt_t  sampleData[4];
bc7215FormatPkt_t   sampleFormat[4];
uint8_t             sampleStatus[4];
const bc7215CombinedMsg_t rcvdMessage[4] = {
	{.bitLen = 0, .body = {.msg = {.fmt = &sampleFormat[0], .datPkt = (bc7215DataVarPkt_t*)&sampleData[0]}}},
	{.bitLen = 0, .body = {.msg = {.fmt = &sampleFormat[1], .datPkt = (bc7215DataVarPkt_t*)&sampleData[1]}}},
	{.bitLen = 0, .body = {.msg = {.fmt = &sampleFormat[2], .datPkt = (bc7215DataVarPkt_t*)&sampleData[2]}}},
	{.bitLen = 0, .body = {.msg = {.fmt = &sampleFormat[3], .datPkt = (bc7215DataVarPkt_t*)&sampleData[3]}}}
};

bc7215DataMaxPkt_t  backupBaseData;
uint8_t             backupStatus;
uint8_t             sampleCount = 0;
bool                acInitOK = false;
bool				formatReceived;
uint32_t            timerStartTime = 0;

// 字符串常量
const char* MODES_STR[] = { "Auto", "Cool", "Heat", "Dry", "Fan", "Keep", "n/a" };
const char* FANS_STR[] = { "Auto", "Low", "Med", "High", "Keep", "n/a" };
const char* PRESSED_KEY[] = { "Temp +", "Temp -", "Mode", "Fan Speed", "Keep" };
const char* POWER_STR[] = { "Off", "On", "Toggle", "n/a" };

// --- 辅助函数 ---

void Clear_Input(void) { UartRing_Clear(); }

// 获取控制台一行输入 (非阻塞，返回 true 表示获取到一行)
bool Get_Input_Line(void)
{
    static uint8_t idx = 0;
    while (UartRing_Available())
    {
        char c = UartRing_Read();
        if (c != '\n' && c != '\r')
        {
            HAL_UART_Transmit(appConfig.consolePort, (uint8_t*)&c, 1, 10);
        }
        if (c == '\n' || c == '\r')
        {
            printf("\r\n");
            inputLine[idx] = 0;
            idx = 0;
            return true;
        }
        else if (idx < sizeof(inputLine) - 1)
        {
            inputLine[idx++] = c;
        }
    }
    return false;
}


/*
 * Display unit menu
 */
void Show_Unit_Menu(void)
{
    printf("\r\nBefore start, please choose your AC's temperature mode:\r\n");
    printf("   1. °C Celsius\r\n");
    printf("   2. °F Fahrenheit\r\n");
    printf("\r\n");
}

/*
 * Display main menu
 */
void Show_Main_Menu(void)
{
    printf("\r\n*****************************************************\r\n");
    printf("* Welcome to BC7215A Universal AC Controller Demo  *\r\n");
    printf("*****************************************************\r\n");
    printf("AC Library Version: ");
    printf(bc7215_ac_get_ver());
    printf("\r\nTemperature Mode: ");
    if (acIsFahrenheit)
    {
    	printf("Fahrenheit\r\n");
    }
    else
    {
    	printf("Celsius\r\n");
    }
    printf("\r\nCurrent AC control library status: ");
    if (acInitOK)
        printf("*** INITIALIZED ***\r\n");
    else
        printf("!!! Not initialized !!! (must be paired with AC before use)\r\n");
    printf("Please select:\r\n");
    printf("   1. Pairing with AC\r\n");
    printf("   2. Control air conditioner\r\n");
    printf("   3. Save pairing data\r\n");
    printf("   4. Read saved data and pair with it\r\n");
    printf("   5. Try next match (if paired successfully but cannot control AC properly)\r\n");
    printf("   6. Load predefined protocol\r\n");
    printf("   7. Parse IR signal\r\n");
    printf("   8. Set Temperature mode\r\n");
    printf("\r\n");
}

/*
 * Display control menu
 */
void Show_Ctrl_Menu(void)
{
    printf("\r\nAC Control, please select:\r\n");
    printf("   1. Change A/C settings\r\n");
    printf("   2. Power on\r\n");
    printf("   3. Power off\r\n");
    printf("   4. Return to upper menu\r\n");
    printf("\r\n");
}

/*
 * Display parameter setting menu
 */
void Show_Param_Menu(void)
{
    printf("*** AC setting adjustment ****\r\n");
    printf("Input format:\r\n");
    if (acIsFahrenheit)
    {
		printf("  Temperature, Mode, Fan, Pressed Key : (e.g. '74, 1, 2, 2')\r\n");
		printf("Fewer parameter is allowed, for example '74, 2' means set to '74°F Heating',\r\nFan Speed and Pressed Key "
			   "unchanged.\r\n");
		printf("Temperature(°F)    Mode           Fan                Key\r\n");
		printf(" Range: 60~88       0 - Auto       0 - Auto           0 - Temp +\r\n");
    }
    else
    {
		printf("  Temperature, Mode, Fan, Pressed Key : (e.g. '24, 1, 2, 2')\r\n");
		printf("Fewer parameter is allowed, for example '18, 2' means set to '18°C Heating',\r\nFan Speed and Pressed Key "
			   "unchanged.\r\n");
		printf("Temperature(°C)    Mode           Fan                Key\r\n");
		printf(" Range: 16~30       0 - Auto       0 - Auto           0 - Temp +\r\n");
    }
    printf("                    1 - Cool       1 - Low            1 - Temp -\r\n");
    printf("                    2 - Heat       2 - Med            2 - Mode\r\n");
    printf("                    3 - Dry        3 - High           3 - Fan\r\n");
    printf("                    4 - Fan\r\n");
    printf(" * Values outside above ranges indicate maintaining current state for that item\r\n");
    printf("------------------------------------------------------------------------------------\r\n");
    printf("(Note: Limited to settings supported by the controlled AC.)\r\n");
    printf("Now please enter AC parameter values: (enter 'exit' to return to upper menu)\r\n");
    printf("\r\n");
}

/*
 * Display predefined protocol menu
 */
void Show_Predef_Menu(void)
{
    printf("\r\n");
    printf("A few protocols are not supported for direct decoding by BC7215A chip.\r\n");
    printf("When direct sampling fails, try using predefined data to control AC.\r\n");
    printf("Please select:\r\n");
    for (int i = 0; i < bc7215_ac_predefined_cnt(); i++)
    {
        printf("  %d. ", i);
        printf(bc7215_ac_predefined_name(i));
        printf("\r\n");
    }
    printf("\r\n");
}

/*
 * Print data in hexadecimal format
 */
void Print_Data(const void* data, uint8_t len)
{
    int i;
    if (data)
    {
        for (i = 0; i < len; i++)
        {
            printf("%02X ", *((uint8_t*)data + i));
        }
        printf("\r\n");
    }
}

// 开始采样
void Ac_StartCapture(uint8_t rx_mode)
{
    sampleCount = 0;
    formatReceived = false;
    bc7215_set_rx();
    HAL_Delay(50);
   	bc7215_set_rx_mode(rx_mode);
    bc7215_clr_data();
    bc7215_clr_format();
}


// 停止采样
void Ac_StopCapture(void)
{
    bc7215_set_tx();
    HAL_Delay(50);
}


// 检查是否已经完成红外信号采集
bool Check_Signal_Captured(void)
{
    if (bc7215_is_busy())
    {        // 如果BC7215A处于忙状态，更新计时器后返回
        timerStartTime = HAL_GetTick();
    }
    else
    {
    	if (bc7215_data_ready())
    	{
    		if (sampleCount < 4)
    		{
				sampleStatus[sampleCount] = bc7215_get_data((bc7215DataVarPkt_t*)&sampleData[sampleCount]);
				if (bc7215_format_ready())
				{
					formatReceived = true;
					bc7215_get_format(&sampleFormat[sampleCount]);
				}
				sampleCount++;
    		}
            timerStartTime = HAL_GetTick();
    	}

        if (sampleCount > 0)
        {        // 如果已经收到了采样数据，有200ms空闲后结束采样
            if (HAL_GetTick() - timerStartTime > 200)
            {
                return true;        // 捕获完成
            }
        }
    }
    return false;
}

// 初始化（配对）,调用时应已取得采样数据
bool Ac_Init(void)
{
    int i, j;
    acInitOK = false;
    if (sampleCount == 1)
    {
    	if (sampleStatus[0]&0x80)	// check error bit
    	{
    		return acInitOK;
    	}
    	if (acIsFahrenheit)
    	{
    		acInitOK = bc7215_ac_init_f(sampleStatus[0], (const bc7215DataVarPkt_t*)(&rcvdMessage[0]));
    	}
    	else
    	{
    		acInitOK = bc7215_ac_init(sampleStatus[0], (const bc7215DataVarPkt_t*)(&rcvdMessage[0]));
    	}
    }
    else if (sampleCount > 1)
    {
        for (j = 0; j < sampleCount; j++)
        {
        	if (sampleStatus[j]&0x80)	// check error bit
        	{
        		return acInitOK;
        	}
            if (sampleStatus[j] & 0x40)        // if receiving status has "REV" bit set, reverse every byte of data
            {
                for (i = 0; i < (sampleData[j].bitLen + 7) / 8; i++)
                {
                    sampleData[j].data[i] = ~sampleData[j].data[i];
                }
                sampleStatus[j] &= 0xbf;
            }
        }
    	if (acIsFahrenheit)
    	{
    		acInitOK = bc7215_ac_init2_f(sampleCount, rcvdMessage, 0);
    	}
    	else
    	{
    		acInitOK = bc7215_ac_init2(sampleCount, rcvdMessage, 0);
    	}
    }
    return acInitOK;
}

// 使用预定义数据初始化
bool Ac_Init_Predef(uint8_t index)
{
    const bc7215DataVarPkt_t* pkt;
    acInitOK = false;
    if (index < bc7215_ac_predefined_cnt())
    {
    	if (acIsFahrenheit)
    	{
    		pkt = bc7215_ac_predefined_data_f(index);
    	}
    	else
    	{
    		pkt = bc7215_ac_predefined_data(index);
    	}
        sampleData[0].bitLen = pkt->bitLen;
        memcpy(sampleData[0].data, pkt->data, (pkt->bitLen + 7) / 8);
        sampleFormat[0] = *bc7215_ac_predefined_fmt(index);
        sampleStatus[0] = sampleFormat[0].signature.bits.sig;

    	if (acIsFahrenheit)
    	{
    		acInitOK = bc7215_ac_init_f(sampleStatus[0], (const bc7215DataVarPkt_t*)&rcvdMessage[0]);
    	}
    	else
    	{
    		acInitOK = bc7215_ac_init(sampleStatus[0], (const bc7215DataVarPkt_t*)&rcvdMessage[0]);
    	}
    }
    return acInitOK;
}

// 解析接收缓冲区中红外信号 (此函数须在成功采样后调用)
bool Ac_Parse(int8_t* temp, int8_t* mode, int8_t* fan, int8_t* power)
{
    int i, j;
    if (sampleCount == 1)
    {
    	if (!(sampleStatus[0]&0x80))	// check error bit
    	{
    		bc7215_ac_replace_base(sampleStatus[0], (const bc7215DataVarPkt_t*)&sampleData[0]);
    	}
    	else
    	{
    		return false;
    	}
    }
    else if (sampleCount > 1)
    {
        for (j = 0; j < sampleCount; j++)
        {
        	if (sampleStatus[j] & 0x80)		// check error bit
        	{
        		return false;
        	}
            if (sampleStatus[j] & 0x40)        // if receiving status has "REV" bit set, reverse every byte of data
            {
                for (i = 0; i < (sampleData[j].bitLen + 7) / 8; i++)
                {
                    sampleData[j].data[i] = ~sampleData[j].data[i];
                }
                sampleStatus[j] &= 0xbf;
            }
        }
        bc7215_ac_replace_base(sampleCount, (const bc7215DataVarPkt_t*)rcvdMessage);
    }
	if (acIsFahrenheit)
	{
		return bc7215_ac_parse_f(temp, mode, fan, power);
	}
	else
	{
		return bc7215_ac_parse(temp, mode, fan, power);
	}
}

// 发送 AC 命令
void Send_AC_Cmd(const bc7215DataVarPkt_t* pkt)
{
    if (pkt == NULL)
        return;

    // 如果 bitLen 为 0，说明是 CombinedMsg (包含格式指针)
    if (pkt->bitLen == 0)
    {
        const bc7215CombinedMsg_t* msg = (const bc7215CombinedMsg_t*)pkt;
        bc7215_load_format(msg->body.msg.fmt);
        bc7215_IR_tx(msg->body.msg.datPkt);
    }
    else
    {
        // 只有数据，加载 Base Format (需确保已 Init)
        bc7215_load_format(bc7215_ac_get_base_fmt());
        bc7215_IR_tx(pkt);
    }
}

// --- 初始化(配对)操作 ---
void Job_Capture(void)
{
    uint8_t        i;

    switch (l2State)
    {
    case STEP1:
    	if (acIsFahrenheit)
    	{
    		printf("\r\nPairing: Please set AC remote to < Cooling mode, 78°F >, and press Enter on keyboard...\r\n");
    	}
    	else
    	{
    		printf("\r\nPairing: Please set AC remote to < Cooling mode, 25°C >, and press Enter on keyboard...\r\n");
    	}
        Clear_Input();
        l2State = STEP2;
        break;
    case STEP2:
        if (Get_Input_Line())
        {
            printf("Now please aim at IR receiver and press < Fan Speed > button on remote. \r\nWill automatically "
                   "proceed to next step after receiving signal...\r\n");
            Ac_StartCapture(1);
            l2State = STEP3;
        }
        break;
    case STEP3:
        if (Check_Signal_Captured())
        {        // 等待采样结束
            Ac_StopCapture();
            printf("Received %d data packet(s): \r\n", sampleCount);
            for (i = 0; i < sampleCount; i++)
            {
                printf("Pkt %d -- status=0x%X, bitLen=0x%X, Data: ", i, sampleStatus[i], sampleData[i].bitLen);
                Print_Data(&sampleData[i].data, (sampleData[i].bitLen + 7) / 8);
            }
            if (formatReceived)
            {
				if (Ac_Init())        // Try to initialize
				{
					printf("Initialization (Pairing)  **SUCCESS** !!! \r\n");
				}
				else        // Initialization failed
				{
					printf("Initialization (Pairing) **FAILED**, "
						   "\r\npossibly due to incorrect remote state settings or receiving decode errors. \r\nPlease "
						   "check remote settings and try again.\r\n");
				}
				l2State = STEP4;
            }
            else
            {
            	printf("BC7215A not in complex mode, trying again...\r\n");
            	l2State = STEP1;
            }
        }
        break;
    case STEP4:
        printf("Now press Enter to continue. \r\n");
        Clear_Input();
        l2State = STEP5;
        break;
    case STEP5:
        if (Get_Input_Line())
        {
            mainState = MAIN_MENU;
            l2State = STEP1;
        }
        break;
    default:
        break;
    }
}

// 控制空调
void Job_AC_Control(void)
{
    const bc7215DataVarPkt_t* pkt = NULL;

    switch (l2State)
    {
    case STEP1:
        Show_Ctrl_Menu();
        Clear_Input();
        l2State = STEP2;
        break;
    case STEP2:
        if (Get_Input_Line())
        {
            if (inputLine[0] == '1')
            {        // Change A/C settings
                Show_Param_Menu();
                Clear_Input();
                l2State = STEP3;
            }
            else if (inputLine[0] == '2')
            {        // Power ON
                printf("Sending A/C ON command...\r\n");
                pkt = bc7215_ac_on();
                if (pkt == NULL)
                    pkt = bc7215_ac_get_base_data();        // 无需专用开机指令
                Print_Data(pkt->data, (pkt->bitLen + 7) / 8);
                timerStartTime = HAL_GetTick();
                Send_AC_Cmd(pkt);
                goBackState = STEP2;
                l2State = STEP4;
            }
            else if (inputLine[0] == '3')
            {        // Power OFF
                printf("Sending A/C OFF command...\r\n");
                pkt = bc7215_ac_off();
                Print_Data(pkt->data, (pkt->bitLen + 7) / 8);
                timerStartTime = HAL_GetTick();
                Send_AC_Cmd(pkt);
                l2State = STEP4;
            }
            else if (inputLine[0] == '4')
            {
                mainState = MAIN_MENU;
                l2State = STEP1;
            }
            Clear_Input();
        }
        break;
    case STEP3:        // 改变空调参数
        if (Get_Input_Line())
        {
            /* 检查是否是 "exit" 命令 (忽略大小写) */
            if (strcasecmp(inputLine, "exit") == 0)
            {
                printf("Exit\r\n");
                l2State = STEP1;
            }
            else
            {
                int t, m, f, k;
                t = -1;
                m = -1;
                f = -1;
                k = -1;
                /* 尝试解析 "温度,模式,风速,按键" 格式 (例如: 26,1,2,2) */
                sscanf(inputLine, "%d,%d,%d,%d", &t, &m, &f, &k);

                /* 范围限制 */
                if (m < 0 || m > 4)
                    m = 5;        // 5 = Keep
                if (f < 0 || f > 3)
                    f = 4;        // 4 = Keep
                if (k < 0 || k > 3)
                    k = 4;        // 4 = Keep

                if (!acIsFahrenheit && (t >= 16 && t <= 30))
                {
                    printf("Sending command to set AC to: %d°C, Mode: %s  Fan: %s, Key Pressed: %s\r\n", t,
                        MODES_STR[m], FANS_STR[f], PRESSED_KEY[k]);
                }
                else if (acIsFahrenheit && (t >= 60 && t <= 88))
                {
                    printf("Sending command to set AC to: %d°F, Mode: %s  Fan: %s, Key Pressed: %s\r\n", t,
                        MODES_STR[m], FANS_STR[f], PRESSED_KEY[k]);

                }
                else
                {
                    printf(
                        "Sending command to set AC to: Temperature unchanged, Mode: %s  Fan: %s, Key Pressed: %s\r\n",
                        MODES_STR[m], FANS_STR[f], PRESSED_KEY[k]);
                }
            	if (acIsFahrenheit)
            	{
					/* 注意: ac_set 期望的温度参数通常是 0-28 (对应 60-88F) */
					pkt = bc7215_ac_set_f(t - 60, m, f, k);
            	}
            	else
            	{
					/* 注意: ac_set 期望的温度参数通常是 0-14 (对应 16-30C) */
					pkt = bc7215_ac_set(t - 16, m, f, k);
            	}

                printf("Sending data: ");
                if (pkt->bitLen != 0)
                {
                    Print_Data(pkt->data, (pkt->bitLen + 7) / 8);
                }
                else
                {
                    pkt = ((bc7215CombinedMsg_t*)pkt)->body.msg.datPkt;
                    Print_Data(pkt->data, (pkt->bitLen + 7) / 8);
                }
                timerStartTime = HAL_GetTick();
                Send_AC_Cmd(pkt);
                goBackState = STEP3;
                l2State = STEP4;
            }
            Clear_Input();
        }
        break;
    case STEP4:        // 等待传输完成
        if (HAL_GetTick() - timerStartTime > 3000)
        {
            printf("Trasmmission timeout.\r\n");
            l2State = goBackState;
        }
        else if (!bc7215_is_busy())
        {
            if (goBackState == STEP3)
            {
                printf("Transmission complete!\r\nInput new value or 'exit' to return to upper level...\r\n");
            }
            else if (goBackState == STEP2)
            {
                printf("Transmission complete!\r\nPlease select next operation...\r\n");
            }
            Clear_Input();
            l2State = goBackState;
        }
        break;
    default:
        break;
    }
}

// 查找下一匹配
void Job_FindNext(void)
{
    switch (l2State)
    {
    case STEP1:
        acInitOK = bc7215_ac_find_next();
        if (acInitOK)
        {
            printf("Next protocol match successful!\r\n");
        }
        else
        {
            printf("No other matching protocols, AC control library needs re-initialization\r\n");
        }
        printf("Press Enter continue...");
        Clear_Input();
        l2State = STEP2;
        break;
    case STEP2:        // Wait for user confirmation
        if (Get_Input_Line())
        {
            mainState = MAIN_MENU;
            l2State = STEP1;
        }
        break;
    default:
        break;
    }
}

// 加载预定义协议
void Job_LoadPredef(void)
{
    uint8_t choice;
    switch (l2State)
    {
    case STEP1:        // Show predefined menu
        Show_Predef_Menu();
        Clear_Input();
        l2State = STEP2;
        break;
    case STEP2:        // Handle user selection
        if (Get_Input_Line())
        {
            choice = atoi(inputLine);
            printf("Selected option :%d\r\n", choice);
            if (Ac_Init_Predef(choice))
            {
                printf("Format: ");
                Print_Data(bc7215_ac_get_base_fmt(), 33);
                printf("Data: ");
                Print_Data(bc7215_ac_get_base_data(), (bc7215_ac_get_base_data()->bitLen + 7) / 8 + 2);
                printf("Initialization ***SUCCESSFUL*** !!!\r\nPress Enter to continue...\r\n");
            }
            else
            {
                printf("Format: ");
                Print_Data(&sampleFormat[0], 33);
                printf("Data: ");
                Print_Data(&sampleData[0], BC7215_MAX_RX_DATA_SIZE);
                printf("Initialization ***FAILED*** !!!\r\nPress Enter to continue...\r\n");
            }
            Clear_Input();
            l2State = STEP3;
        }
        break;
    case STEP3:        // Wait for user confirmation
        if (Get_Input_Line())
        {
            mainState = MAIN_MENU;
            l2State = STEP1;
        }
        break;
    default:
        break;
    }
}

// 红外指令解析
void Job_Parsing()
{
    int8_t T, M, F, P;
    switch (l2State)
    {
    case STEP1:
        if (acInitOK)
        {
            printf("\n\nReceiving IR signal and parse(decode) Temperature, Mode, Fan Speed, and Power status from "
                   "it.\r\n");
            printf("\r\nBC7215A is now in RX mode, ready to decode. Press Enter exit\r\n");

            Ac_StartCapture(0);
            Clear_Input();
            l2State = STEP2;
        }
        else
        {
            printf("\n\nThis function is only available when it's paired with AC\r\n");
            printf("Please pair first.\r\n");
            mainState = MAIN_MENU;
            l2State = STEP1;
        }
        break;
    case STEP2:        // Wait for IR signal
        if (Check_Signal_Captured())
        {
            Ac_StopCapture();
            T = -1;
            M = -1;
            F = -1;
            P = -1;
            if (Ac_Parse(&T, &M, &F, &P))
            {
                if ((M < 0) || (M > 4))
                    M = 6;
                if ((F < 0) || (F > 3))
                    F = 5;
                if ((P < 0) || (P > 2))
                    P = 3;
            	if (acIsFahrenheit)
            	{
					if ((T >= 0) && (T <= 28))
						T += 60;
					printf("Parsing Result: Temp: %d°F,  Mode: ", T);
            	}
            	else
            	{
					if ((T >= 0) && (T <= 14))
						T += 16;
					printf("Parsing Result: Temp: %d°C,  Mode: ", T);
            	}
                printf(MODES_STR[M]);
                printf(",  Fan Speed: ");
                printf(FANS_STR[F]);
                printf(",  Power: ");
                printf(POWER_STR[P]);
                printf("\r\n");
            }
            else
            {
                printf("Parsing failed, please check if the IR signal is from the same remote.\r\n");
            }
            Ac_StartCapture(0);
        }
        if (Get_Input_Line())
        {
            Ac_StopCapture();
            bc7215_ac_replace_base(backupStatus, (bc7215DataVarPkt_t*)&backupBaseData);        // 恢复为原来的基础数据包
            mainState = MAIN_MENU;
            l2State = STEP1;
        }
        break;
    default:
        break;
    }
}

void Job_Backup(void)
{
    switch (l2State)
    {
    case STEP1:
        /*
         * 此部分代码需用户根据具体系统添加，所需保存数据包括(假设已正确配对)：
         * bc7215_ac_get_base_data()所返回的数据包
         * bc7215_ac_get_base_fmt()所返回的格式包(如初始化操作已使用包含格式信息的数据的参数)
         * 调用bc7215_ac_find_next()的次数
         * 如果为有按键需要额外采样的协议，还需要保存：
         * bc7215_ac_get_2nd_base()所返回的复合格式combinedMsg_t所指向的数据包和格式包
         */
        printf("This part of code need to be implemented by user according to the system environment.\r\nPress Enter "
               "to continue...\r\n");
        l2State = STEP2;
        Clear_Input();
        break;
    case STEP2:
        if (Get_Input_Line())
        {
            mainState = MAIN_MENU;
            l2State = STEP1;
            Clear_Input();
        }
        break;
    default:
        break;
    }
}

void Job_Restore(void)
{
    switch (l2State)
    {
    case STEP1:
        /*
         * 此部分代码需用户根据具体系统添加，需要读出Job_Backup()中所保存的全部数据，并按下面步骤初始化(配对)
         * 1.使用基础格式包中的Format.signature.bits.sig 作为Status，用所保存的基础格式包和基础数据包
         *   组成复合数据combinedMsg_t，调用bc7215_ac_init()完成初始化
         * 2.执行与保存数据相同多次数的bc7215_ac_find_next()
         * 3.如果保存数据中还有额外采样的格式和数据，使用bc7215_ac_save_2nd_base()将其加载到码库
         */
        printf("This part of code need to be implemented by user according to the system environment.\r\nPress Enter "
               "to continue...\r\n");
        l2State = STEP2;
        Clear_Input();
        break;
    case STEP2:
        if (Get_Input_Line())
        {
            mainState = MAIN_MENU;
            l2State = STEP1;
            Clear_Input();
        }
        break;
    default:
        break;
    }
}

// 主菜单处理
void Job_MainMenu(void)
{
    switch (l2State)
    {
    case STEP1:
        Show_Main_Menu();
        Clear_Input();
        l2State = STEP2;
        break;
    case STEP2:
        if (Get_Input_Line())
        {        // 如果有输入
            switch (inputLine[0])
            {
            case '1':        // 初始化（配对）
                mainState = CAPTURE;
                l2State = STEP1;
                break;
            case '2':        // Control AC
                if (acInitOK)
                {
                    mainState = AC_CONTROL;
                    l2State = STEP1;
                }
                else
                {
                    printf("\r\nAC control library not initialized yet, please pair first\r\n");
                }
                break;
            case '3':        // Save data
                if (acInitOK)
                {
                    mainState = BACKUP;
                    l2State = STEP1;
                }
                else
                {
                    printf("\r\nAC control library not initialized yet, data can only be saved after successful "
                           "pairing.\r\n");
                }
                break;
            case '4':        // Restore data
                mainState = RESTORE;
                l2State = STEP1;
                break;
            case '5':        // Find next protocol
                if (acInitOK)
                {
                    mainState = FIND_NEXT;
                    l2State = STEP1;
                }
                else
                {
                    printf("\r\nThis function is only available after pairing\r\n");
                }
                break;
            case '6':        // Load predefined
                mainState = LOAD_PREDEF;
                l2State = STEP1;
                break;
            case '7':        // Parsing
                if (acInitOK)
                {
                    // 因为解析操作会替换基础数据包，故先备份下来，以便恢复
                    backupBaseData.bitLen = bc7215_ac_get_base_data()->bitLen;
                    memcpy(backupBaseData.data, bc7215_ac_get_base_data()->data, (backupBaseData.bitLen + 7) / 8);
                    backupStatus = bc7215_ac_get_base_fmt()->signature.bits.sig;
                    mainState = IR_PARSING;
                    l2State = STEP1;
                }
                else
                {
                    printf("\r\nThis function is only available after pairing\r\n");
                }
                break;
            case '8':		// Set Temp Mode
            	mainState = CHOOSE_UNIT;
            	l2State = STEP1;
            	break;
            default:
                mainState = MAIN_MENU;
                l2State = STEP1;
                break;
            }
            Clear_Input();
        }
        break;
    default:
        break;
    }
}

void Job_ChooseUnit(void)
{
    switch (l2State)
    {
    case STEP1:
    	Show_Unit_Menu();
        Clear_Input();
        l2State = STEP2;
        break;
    case STEP2:
        if (Get_Input_Line())
        {        // 如果有输入
            switch (inputLine[0])
            {
            case '1':        // 摄氏
            	acIsFahrenheit = false;
            	printf("\r\nYour AC is Celsius. ");
            	l2State = STEP3;
            	break;
            case '2':		// 华氏
            	acIsFahrenheit = true;
            	printf("\r\nYour AC is Fahrenheit. ");
            	l2State = STEP3;
            	break;
            default:
            	break;
            }
        }
        break;
    case STEP3:
    	mainState = MAIN_MENU;
    	l2State = STEP1;
    	break;
    default:
    	break;
    }
}

// --- 主程序入口 ---

void App_Init(appConfig_t* hwConfig)
{
    // 初始化终端接口
	UartRing_Init(hwConfig->consolePort);

    // 初始化胶水层 (硬件适配层)
    BC7215_Glue_Init(hwConfig->bc7215Port, hwConfig->modPort, hwConfig->modPin, hwConfig->busyPort, hwConfig->busyPin);

    // 初始进入发送模式
    bc7215_set_tx();

    printf("System Initialized.\r\n");
}

void App_Loop(void)
{
    switch (mainState)
    {
    case CHOOSE_UNIT:
    	Job_ChooseUnit();
    	break;
    case MAIN_MENU:
        Job_MainMenu();
        break;
    case CAPTURE:
        Job_Capture();
        break;
    case AC_CONTROL:
        Job_AC_Control();
        break;
    case BACKUP:
        Job_Backup();
        break;
    case RESTORE:
        Job_Restore();
        break;
    case FIND_NEXT:
        Job_FindNext();
        break;
    case LOAD_PREDEF:
        Job_LoadPredef();
        break;
    case IR_PARSING:
        Job_Parsing();
        break;
    default:
        mainState = MAIN_MENU;
        break;
    }
}
