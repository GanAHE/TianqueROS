@Echo off

REM apps/tools/mkkconfig.bat
REM
REM   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
REM   Author: Gregory Nutt <gnutt@nuttx.org>
REM
REM Redistribution and use in source and binary forms, with or without
REM modification, are permitted provided that the following conditions
REM are met:
REM
REM 1. Redistributions of source code must retain the above copyright
REM    notice, this list of conditions and the following disclaimer.
REM 2. Redistributions in binary form must reproduce the above copyright
REM    notice, this list of conditions and the following disclaimer in
REM    the documentation and/or other materials provided with the
REM    distribution.
REM 3. Neither the name NuttX nor the names of its contributors may be
REM    used to endorse or promote products derived from this software
REM    without specific prior written permission.
REM
REM THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
REM "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
REM LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
REM FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
REM COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
REM INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
REM BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
REM OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
REM AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
REM LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
REM ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
REM POSSIBILITY OF SUCH DAMAGE.
REM

REM Parse command line arguments

SET menu=""
SET kconfig=Kconfig

:ArgLoop
IF "%1"=="" GOTO :EndOfLoop
IF "%1"=="-m" GOTO :SetMenu
IF "%1"=="-o" GOTO :SetKconfig
IF "%1"=="-h" GOTO :ShowUsage

Echo ERROR: Unrecogized option %1
GOTO :ShowUsage

:SetDebug
SET debug=-d
GOTO :NextArg

:SetMenu
SHIFT
SET menu=%1
GOTO :NextArg

:SetKconfig
SHIFT
SET kconfig=%1

:NextArg
SHIFT
GOTO :ArgLoop

REM Check input Parameters

:EndOfLoop
IF EXIST %kconfig% (
  Del /f /q %kconfig%
REM   IF %ERRORLEVEL% GTR 0 (
REM     Echo ERROR: failed to remove %kconfig%
REM     GOTO :End
REM   )
)

REM Get the current directory
SET APPSDIR=%cd%
SET APPSDIR=%APPSDIR:\=/%

Echo # > %kconfig%
Echo # For a description of the syntax of this configuration file, >> %kconfig%
Echo # see the file kconfig-language.txt in the NuttX tools repository. >> %kconfig%
Echo # >> %kconfig%
Echo # This file is autogenerated, do not edit. >> %kconfig%
Echo # >> %kconfig%
Echo[ >> %kconfig%

IF %menu% NEQ "" (
  Echo menu %menu% >> %kconfig%
)

DIR /B /A:D >_tmp_.dat

FOR /F "tokens=*" %%s IN (_tmp_.dat) do (
  IF EXIST %%s\Kconfig (
    Echo source "%APPSDIR%/%%s/Kconfig" >> %kconfig%
  )
)
DEL _tmp_.dat

IF %menu% NEQ "" (
  Echo endmenu # %menu% >> %kconfig%
)

GOTO :End

REM Exit showing usage

:ShowUsage
Echo USAGE: %0  [-d] [-m ^<menu^>] [-o ^<kconfig-file^>]
Echo        %0  [-h]
Echo Where:
Echo  ^<-d^>:
Echo    Enables debug output
Echo  -m ^<menu^>:
Echo    Menu description
Echo  -o ^<kconfig-file^>:
Echo    Identifies the specific configuratin for the selected ^<board-name^>.
Echo    This must correspond to a sub-directory under the board directory at
Echo    under nuttx/boards/^<arch^>/^<chip^>/^<board^>/.
Echo  ^<-h^>:
Echo    Prints this message and exits.

REM Exit

:End

