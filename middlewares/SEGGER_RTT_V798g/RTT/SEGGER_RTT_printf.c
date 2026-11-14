/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2021 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER RTT * Real Time Transfer for embedded targets         *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the RTT protocol and J-Link.                       *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
*                                                                    *
*       RTT version: 7.98g                                           *
*                                                                    *
**********************************************************************

---------------------------END-OF-HEADER------------------------------
File    : SEGGER_RTT_printf.c
Purpose : Replacement for printf to write formatted data via RTT
Revision: $Rev: 17697 $
----------------------------------------------------------------------
*/
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/

#ifndef SEGGER_RTT_PRINTF_BUFFER_SIZE
  #define SEGGER_RTT_PRINTF_BUFFER_SIZE (64)
#endif

#include <stdlib.h>
#include <stdarg.h>


#define FORMAT_FLAG_LEFT_JUSTIFY   (1u << 0)
#define FORMAT_FLAG_PAD_ZERO       (1u << 1)
#define FORMAT_FLAG_PRINT_SIGN     (1u << 2)
#define FORMAT_FLAG_ALTERNATE      (1u << 3)
#define FORMAT_FLAG_SCIENTIFIC     (1u << 4)  // 新增：科学计数法标志
#define FORMAT_FLAG_PRINT_SPACE    (1u << 5)  // 新增：正数前显示空格
/*********************************************************************
*
*       Types
*
**********************************************************************
*/

typedef struct {
  char*     pBuffer;
  unsigned  BufferSize;
  unsigned  Cnt;

  int   ReturnValue;

  unsigned RTTBufferIndex;
} SEGGER_RTT_PRINTF_DESC;

/*********************************************************************
*
*       Function prototypes
*
**********************************************************************
*/

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       _StoreChar
*/
static void _StoreChar(SEGGER_RTT_PRINTF_DESC * p, char c) {
  unsigned Cnt;

  Cnt = p->Cnt;
  if ((Cnt + 1u) <= p->BufferSize) {
    *(p->pBuffer + Cnt) = c;
    p->Cnt = Cnt + 1u;
    p->ReturnValue++;
  }
  //
  // Write part of string, when the buffer is full
  //
  if (p->Cnt == p->BufferSize) {
    if (SEGGER_RTT_Write(p->RTTBufferIndex, p->pBuffer, p->Cnt) != p->Cnt) {
      p->ReturnValue = -1;
    } else {
      p->Cnt = 0u;
    }
  }
}

/*********************************************************************
*
*       _PrintUnsigned
*/
static void _PrintUnsigned(SEGGER_RTT_PRINTF_DESC * pBufferDesc, unsigned v, unsigned Base, unsigned NumDigits, unsigned FieldWidth, unsigned FormatFlags) {
  static const char _aV2C[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  unsigned Div;
  unsigned Digit;
  unsigned Number;
  unsigned Width;
  char c;

  Number = v;
  Digit = 1u;
  //
  // Get actual field width
  //
  Width = 1u;
  while (Number >= Base) {
    Number = (Number / Base);
    Width++;
  }
  if (NumDigits > Width) {
    Width = NumDigits;
  }
  //
  // Print leading chars if necessary
  //
  if ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u) {
    if (FieldWidth != 0u) {
      if (((FormatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && (NumDigits == 0u)) {
        c = '0';
      } else {
        c = ' ';
      }
      while ((FieldWidth != 0u) && (Width < FieldWidth)) {
        FieldWidth--;
        _StoreChar(pBufferDesc, c);
        if (pBufferDesc->ReturnValue < 0) {
          break;
        }
      }
    }
  }
  if (pBufferDesc->ReturnValue >= 0) {
    //
    // Compute Digit.
    // Loop until Digit has the value of the highest digit required.
    // Example: If the output is 345 (Base 10), loop 2 times until Digit is 100.
    //
    while (1) {
      if (NumDigits > 1u) {       // User specified a min number of digits to print? => Make sure we loop at least that often, before checking anything else (> 1 check avoids problems with NumDigits being signed / unsigned)
        NumDigits--;
      } else {
        Div = v / Digit;
        if (Div < Base) {        // Is our divider big enough to extract the highest digit from value? => Done
          break;
        }
      }
      Digit *= Base;
    }
    //
    // Output digits
    //
    do {
      Div = v / Digit;
      v -= Div * Digit;
      _StoreChar(pBufferDesc, _aV2C[Div]);
      if (pBufferDesc->ReturnValue < 0) {
        break;
      }
      Digit /= Base;
    } while (Digit);
    //
    // Print trailing spaces if necessary
    //
    if ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == FORMAT_FLAG_LEFT_JUSTIFY) {
      if (FieldWidth != 0u) {
        while ((FieldWidth != 0u) && (Width < FieldWidth)) {
          FieldWidth--;
          _StoreChar(pBufferDesc, ' ');
          if (pBufferDesc->ReturnValue < 0) {
            break;
          }
        }
      }
    }
  }
}

/*********************************************************************
*
*       _PrintInt
*/
static void _PrintInt(SEGGER_RTT_PRINTF_DESC * pBufferDesc, int v, unsigned Base, unsigned NumDigits, unsigned FieldWidth, unsigned FormatFlags) {
  unsigned Width;
  int Number;

  Number = (v < 0) ? -v : v;

  //
  // Get actual field width
  //
  Width = 1u;
  while (Number >= (int)Base) {
    Number = (Number / (int)Base);
    Width++;
  }
  if (NumDigits > Width) {
    Width = NumDigits;
  }
  if ((FieldWidth > 0u) && ((v < 0) || ((FormatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN))) {
    FieldWidth--;
  }

  //
  // Print leading spaces if necessary
  //
  if ((((FormatFlags & FORMAT_FLAG_PAD_ZERO) == 0u) || (NumDigits != 0u)) && ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u)) {
    if (FieldWidth != 0u) {
      while ((FieldWidth != 0u) && (Width < FieldWidth)) {
        FieldWidth--;
        _StoreChar(pBufferDesc, ' ');
        if (pBufferDesc->ReturnValue < 0) {
          break;
        }
      }
    }
  }
  //
  // Print sign if necessary
  //
  if (pBufferDesc->ReturnValue >= 0) {
    if (v < 0) {
      v = -v;
      _StoreChar(pBufferDesc, '-');
    } else if ((FormatFlags & FORMAT_FLAG_PRINT_SIGN) == FORMAT_FLAG_PRINT_SIGN) {
      _StoreChar(pBufferDesc, '+');
    } else {

    }
    if (pBufferDesc->ReturnValue >= 0) {
      //
      // Print leading zeros if necessary
      //
      if (((FormatFlags & FORMAT_FLAG_PAD_ZERO) == FORMAT_FLAG_PAD_ZERO) && ((FormatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0u) && (NumDigits == 0u)) {
        if (FieldWidth != 0u) {
          while ((FieldWidth != 0u) && (Width < FieldWidth)) {
            FieldWidth--;
            _StoreChar(pBufferDesc, '0');
            if (pBufferDesc->ReturnValue < 0) {
              break;
            }
          }
        }
      }
      if (pBufferDesc->ReturnValue >= 0) {
        //
        // Print number without sign
        //
        _PrintUnsigned(pBufferDesc, (unsigned)v, Base, NumDigits, FieldWidth, FormatFlags);
      }
    }
  }
}

/*********************************************************************
*
*       _PrintString
*
* 功能描述: 辅助函数 - 打印字符串
*/
static void _PrintString(SEGGER_RTT_PRINTF_DESC * pBufferDesc, const char* str, 
                        unsigned len, unsigned fieldWidth, unsigned formatFlags) {
    // 计算字符串长度
    if (len == 0) {
        const char* p = str;
        while (*p) {
            len++;
            p++;
        }
    }
    
    // 左对齐前导空格
    if ((formatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0) {
        while (fieldWidth > len) {
            _StoreChar(pBufferDesc, ' ');
            if (pBufferDesc->ReturnValue < 0) return;
            fieldWidth--;
        }
    }
    
    // 打印字符串内容
    while (*str && len > 0) {
        _StoreChar(pBufferDesc, *str);
        if (pBufferDesc->ReturnValue < 0) return;
        str++;
        len--;
    }
    
    // 右对齐后导空格
    if (formatFlags & FORMAT_FLAG_LEFT_JUSTIFY) {
        while (fieldWidth > 0) {
            _StoreChar(pBufferDesc, ' ');
            if (pBufferDesc->ReturnValue < 0) return;
            fieldWidth--;
        }
    }
}

/*********************************************************************
*
*       _PrintFixedPoint
*
* 功能描述: 固定小数点格式打印
*/
static void _PrintFixedPoint(SEGGER_RTT_PRINTF_DESC * pBufferDesc, float value, 
                            unsigned precision, unsigned fieldWidth, unsigned formatFlags) {
    int isNegative = (value < 0.0f);
    float absValue = isNegative ? -value : value;
    
    // 处理符号
    if (isNegative) {
        _StoreChar(pBufferDesc, '-');
        if (pBufferDesc->ReturnValue < 0) return;
        if (fieldWidth > 0) fieldWidth--;
    } else if (formatFlags & FORMAT_FLAG_PRINT_SIGN) {
        _StoreChar(pBufferDesc, '+');
        if (pBufferDesc->ReturnValue < 0) return;
        if (fieldWidth > 0) fieldWidth--;
    } else if (formatFlags & FORMAT_FLAG_PRINT_SPACE) {
        _StoreChar(pBufferDesc, ' ');
        if (pBufferDesc->ReturnValue < 0) return;
        if (fieldWidth > 0) fieldWidth--;
    }
    
    // 分离整数和小数部分
    unsigned intPart = (unsigned int)absValue;
    float fracPart = absValue - intPart;
    
    // 计算整数部分位数
    unsigned temp = intPart;
    unsigned intWidth = 1;
    while (temp >= 10) {
        temp /= 10;
        intWidth++;
    }
    if (intPart == 0) intWidth = 1;
    
    // 计算总宽度
    unsigned totalWidth = intWidth;
    if (precision > 0) {
        totalWidth += 1 + precision; // 小数点 + 小数位数
    }
    
    // 左对齐前导空格
    if ((formatFlags & FORMAT_FLAG_LEFT_JUSTIFY) == 0) {
        while (fieldWidth > totalWidth) {
            if ((formatFlags & FORMAT_FLAG_PAD_ZERO) && precision == 0) {
                _StoreChar(pBufferDesc, '0');
            } else {
                _StoreChar(pBufferDesc, ' ');
            }
            if (pBufferDesc->ReturnValue < 0) return;
            fieldWidth--;
        }
    }
    
    // 打印整数部分
    _PrintUnsigned(pBufferDesc, intPart, 10, 0, 0, formatFlags);
    if (pBufferDesc->ReturnValue < 0) return;
    
    // 打印小数部分（如果精度>0）
    if (precision > 0) {
        _StoreChar(pBufferDesc, '.');
        if (pBufferDesc->ReturnValue < 0) return;
        
        // 处理小数部分
        for (unsigned i = 0; i < precision; i++) {
            fracPart *= 10.0f;
            unsigned digit = (unsigned)fracPart;
            fracPart -= digit;
            
            _StoreChar(pBufferDesc, '0' + digit);
            if (pBufferDesc->ReturnValue < 0) return;
        }
    }
    
    // 右对齐后导空格
    if (formatFlags & FORMAT_FLAG_LEFT_JUSTIFY) {
        while (fieldWidth > totalWidth) {
            _StoreChar(pBufferDesc, ' ');
            if (pBufferDesc->ReturnValue < 0) return;
            fieldWidth--;
        }
    }
}

/*********************************************************************
*
*       _PrintScientific
*
* 功能描述: 科学计数法格式打印
*/
static void _PrintScientific(SEGGER_RTT_PRINTF_DESC * pBufferDesc, float value, 
                            unsigned precision, unsigned fieldWidth, unsigned formatFlags) {
    int isNegative = (value < 0.0f);
    float absValue = isNegative ? -value : value;
    
    // 处理符号
    if (isNegative) {
        _StoreChar(pBufferDesc, '-');
        if (pBufferDesc->ReturnValue < 0) return;
        if (fieldWidth > 0) fieldWidth--;
    } else if (formatFlags & FORMAT_FLAG_PRINT_SIGN) {
        _StoreChar(pBufferDesc, '+');
        if (pBufferDesc->ReturnValue < 0) return;
        if (fieldWidth > 0) fieldWidth--;
    } else if (formatFlags & FORMAT_FLAG_PRINT_SPACE) {
        _StoreChar(pBufferDesc, ' ');
        if (pBufferDesc->ReturnValue < 0) return;
        if (fieldWidth > 0) fieldWidth--;
    }
    
    int exponent = 0;
    float normalizedValue = absValue;
    
    // 规范化数值到 [1.0, 10.0) 范围
    if (normalizedValue != 0.0f) {
        while (normalizedValue >= 10.0f) {
            normalizedValue /= 10.0f;
            exponent++;
        }
        while (normalizedValue < 1.0f) {
            normalizedValue *= 10.0f;
            exponent--;
        }
    }
    
    // 打印规范化后的数值
    unsigned intPart = (unsigned)normalizedValue;
    float fracPart = normalizedValue - intPart;
    
    _StoreChar(pBufferDesc, '0' + intPart);
    if (pBufferDesc->ReturnValue < 0) return;
    
    if (precision > 0) {
        _StoreChar(pBufferDesc, '.');
        if (pBufferDesc->ReturnValue < 0) return;
        
        for (unsigned i = 0; i < precision; i++) {
            fracPart *= 10.0f;
            unsigned digit = (unsigned)fracPart;
            fracPart -= digit;
            
            _StoreChar(pBufferDesc, '0' + digit);
            if (pBufferDesc->ReturnValue < 0) return;
        }
    }
    
    // 打印指数部分
    _StoreChar(pBufferDesc, (formatFlags & FORMAT_FLAG_SCIENTIFIC) ? 'e' : 'E');
    if (pBufferDesc->ReturnValue < 0) return;
    
    if (exponent < 0) {
        _StoreChar(pBufferDesc, '-');
        exponent = -exponent;
    } else {
        _StoreChar(pBufferDesc, '+');
    }
    if (pBufferDesc->ReturnValue < 0) return;
    
    // 指数总是2位数字
    if (exponent < 10) {
        _StoreChar(pBufferDesc, '0');
        if (pBufferDesc->ReturnValue < 0) return;
    }
    _PrintUnsigned(pBufferDesc, exponent, 10, 0, 0, 0);
}
/*********************************************************************
*
*       _PrintFloat
*
* 功能描述: 格式化打印浮点数
*/
static void _PrintFloat(SEGGER_RTT_PRINTF_DESC * pBufferDesc, float value, 
                       unsigned precision, unsigned fieldWidth, unsigned formatFlags) {
    // 处理特殊值
    if (value != value) {  // NaN
        _PrintString(pBufferDesc, "nan", 3, fieldWidth, formatFlags);
        return;
    }
    
    if (value * 2 == value && value != 0.0f) {  // Inf
        if (value < 0.0f) {
            _StoreChar(pBufferDesc, '-');
            if (pBufferDesc->ReturnValue < 0) return;
            if (fieldWidth > 0) fieldWidth--;
        }
        _PrintString(pBufferDesc, "inf", 3, fieldWidth, formatFlags);
        return;
    }
    
    // 检查科学计数法标志
    if (formatFlags & FORMAT_FLAG_SCIENTIFIC) {
        _PrintScientific(pBufferDesc, value, precision, fieldWidth, formatFlags);
    } else {
        _PrintFixedPoint(pBufferDesc, value, precision, fieldWidth, formatFlags);
    }
}

/*********************************************************************
*
*       _PrintFloatGeneral
*
* 功能描述: 通用格式打印（自动选择f或e格式）
*/
static void _PrintFloatGeneral(SEGGER_RTT_PRINTF_DESC * pBufferDesc, float value, 
                              unsigned precision, unsigned fieldWidth, unsigned formatFlags) {
    float absValue = (value < 0.0f) ? -value : value;
    
    // 自动选择格式：小数值用f，大/小数值用e
    if (absValue == 0.0f || (absValue >= 0.0001f && absValue < 1000000.0f)) {
        // 使用固定小数点格式
        _PrintFixedPoint(pBufferDesc, value, precision, fieldWidth, formatFlags);
    } else {
        // 使用科学计数法
        if (formatFlags & FORMAT_FLAG_ALTERNATE) {
            formatFlags |= FORMAT_FLAG_SCIENTIFIC;
        }
        _PrintScientific(pBufferDesc, value, precision, fieldWidth, formatFlags);
    }
}
/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       SEGGER_RTT_vprintf
*
*  Function description
*    Stores a formatted string in SEGGER RTT control block.
*    This data is read by the host.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used. (e.g. 0 for "Terminal")
*    sFormat      Pointer to format string
*    pParamList   Pointer to the list of arguments for the format string
*
*  Return values
*    >= 0:  Number of bytes which have been stored in the "Up"-buffer.
*     < 0:  Error
*/
int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList) {
  char c;
  SEGGER_RTT_PRINTF_DESC BufferDesc;
  int v;
  double f;
  unsigned NumDigits;
  unsigned FormatFlags;
  unsigned FieldWidth;
  char acBuffer[SEGGER_RTT_PRINTF_BUFFER_SIZE];

  BufferDesc.pBuffer        = acBuffer;
  BufferDesc.BufferSize     = SEGGER_RTT_PRINTF_BUFFER_SIZE;
  BufferDesc.Cnt            = 0u;
  BufferDesc.RTTBufferIndex = BufferIndex;
  BufferDesc.ReturnValue    = 0;

  do {
    c = *sFormat;
    sFormat++;
    if (c == 0u) {
      break;
    }
    if (c == '%') {
      //
      // Filter out flags
      //
      FormatFlags = 0u;
      v = 1;
      do {
        c = *sFormat;
        switch (c) {
        case '-': FormatFlags |= FORMAT_FLAG_LEFT_JUSTIFY; sFormat++; break;
        case '0': FormatFlags |= FORMAT_FLAG_PAD_ZERO;     sFormat++; break;
        case '+': FormatFlags |= FORMAT_FLAG_PRINT_SIGN;   sFormat++; break;
        case '#': FormatFlags |= FORMAT_FLAG_ALTERNATE;    sFormat++; break;
        default:  v = 0; break;
        }
      } while (v);
      //
      // filter out field with
      //
      FieldWidth = 0u;
      do {
        c = *sFormat;
        if ((c < '0') || (c > '9')) {
          break;
        }
        sFormat++;
        FieldWidth = (FieldWidth * 10u) + ((unsigned)c - '0');
      } while (1);

      //
      // Filter out precision (number of digits to display)
      //
      NumDigits = 0u;
      c = *sFormat;
      if (c == '.') {
        sFormat++;
        do {
          c = *sFormat;
          if ((c < '0') || (c > '9')) {
            break;
          }
          sFormat++;
          NumDigits = NumDigits * 10u + ((unsigned)c - '0');
        } while (1);
      }
      //
      // Filter out length modifier
      //
      c = *sFormat;
      do {
        if ((c == 'l') || (c == 'h')) {
          sFormat++;
          c = *sFormat;
        } else {
          break;
        }
      } while (1);
      //
      // Handle specifiers
      //
      switch (c) {
      case 'c': {
        char c0;
        v = va_arg(*pParamList, int);
        c0 = (char)v;
        _StoreChar(&BufferDesc, c0);
        break;
      }
      case 'd':
        v = va_arg(*pParamList, int);
        _PrintInt(&BufferDesc, v, 10u, NumDigits, FieldWidth, FormatFlags);
        break;
      case 'u':
        v = va_arg(*pParamList, int);
        _PrintUnsigned(&BufferDesc, (unsigned)v, 10u, NumDigits, FieldWidth, FormatFlags);
        break;
      case 'x':
      case 'X':
        v = va_arg(*pParamList, int);
        _PrintUnsigned(&BufferDesc, (unsigned)v, 16u, NumDigits, FieldWidth, FormatFlags);
        break;
      case 's':
        {
          const char * s = va_arg(*pParamList, const char *);
          if (s == NULL) {
            s = "(NULL)";  // Print (NULL) instead of crashing or breaking, as it is more informative to the user.
          }
          do {
            c = *s;
            s++;
            if (c == '\0') {
              break;
            }
           _StoreChar(&BufferDesc, c);
          } while (BufferDesc.ReturnValue >= 0);
        }
        break;
      case 'p':
        v = va_arg(*pParamList, int);
        _PrintUnsigned(&BufferDesc, (unsigned)v, 16u, 8u, 8u, 0u);
        break;
      case 'f':
      case 'F':
        f = va_arg(*pParamList, double);
        _PrintFloat(&BufferDesc, (float)f, NumDigits, FieldWidth, FormatFlags);
        break;
      case 'e':
      case 'E':
        f = va_arg(*pParamList, double);
        FormatFlags |= FORMAT_FLAG_SCIENTIFIC;
        _PrintFloat(&BufferDesc, (float)f, NumDigits, FieldWidth, FormatFlags);
        break;
      case 'g':
      case 'G':
        f = va_arg(*pParamList, double);
        _PrintFloatGeneral(&BufferDesc, (float)f, NumDigits, FieldWidth, FormatFlags);
        break;
      case '%':
        _StoreChar(&BufferDesc, '%');
        break;
      default:
        break;
      }
      sFormat++;
    } else {
      _StoreChar(&BufferDesc, c);
    }
  } while (BufferDesc.ReturnValue >= 0);

  if (BufferDesc.ReturnValue > 0) {
    //
    // Write remaining data, if any
    //
    if (BufferDesc.Cnt != 0u) {
      SEGGER_RTT_Write(BufferIndex, acBuffer, BufferDesc.Cnt);
    }
    BufferDesc.ReturnValue += (int)BufferDesc.Cnt;
  }
  return BufferDesc.ReturnValue;
}

/*********************************************************************
*
*       SEGGER_RTT_printf
*
*  Function description
*    Stores a formatted string in SEGGER RTT control block.
*    This data is read by the host.
*
*  Parameters
*    BufferIndex  Index of "Up"-buffer to be used. (e.g. 0 for "Terminal")
*    sFormat      Pointer to format string, followed by the arguments for conversion
*
*  Return values
*    >= 0:  Number of bytes which have been stored in the "Up"-buffer.
*     < 0:  Error
*
*  Notes
*    (1) Conversion specifications have following syntax:
*          %[flags][FieldWidth][.Precision]ConversionSpecifier
*    (2) Supported flags:
*          -: Left justify within the field width
*          +: Always print sign extension for signed conversions
*          0: Pad with 0 instead of spaces. Ignored when using '-'-flag or precision
*        Supported conversion specifiers:
*          c: Print the argument as one char
*          d: Print the argument as a signed integer
*          u: Print the argument as an unsigned integer
*          x: Print the argument as an hexadecimal integer
*          s: Print the string pointed to by the argument
*          p: Print the argument as an 8-digit hexadecimal integer. (Argument shall be a pointer to void.)
*/
int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...) {
  int r;
  va_list ParamList;

  va_start(ParamList, sFormat);
  r = SEGGER_RTT_vprintf(BufferIndex, sFormat, &ParamList);
  va_end(ParamList);
  return r;
}
/*************************** End of file ****************************/
