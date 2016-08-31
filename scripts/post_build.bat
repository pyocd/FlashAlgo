:: FlashAlgo
:: Copyright (c) 2011-2015 ARM Limited
::
:: Licensed under the Apache License, Version 2.0 (the "License");
:: you may not use this file except in compliance with the License.
:: You may obtain a copy of the License at
::
::    http://www.apache.org/licenses/LICENSE-2.0
::
:: Unless required by applicable law or agreed to in writing, software
:: distributed under the License is distributed on an "AS IS" BASIS,
:: WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
:: See the License for the specific language governing permissions and
:: limitations under the License.
::
::
:: 2 user commands are not enough so this allows passing the control string
::

set KEIL_ARM=%1
REM make sure fromelf is part of path
set path=%KEIL_ARM%\ARMCC\bin;%path%
set base_path=%2
set trgt_name=%3
set base_name=%base_path%%trgt_name%
fromelf --bin %base_name%.axf -o %base_name%
fromelf --text -s %base_name%.axf -o %base_name%\symbols
armar --create %base_name%\%trgt_name%.ar %base_path%*.o
set SCRIPTS=..\..\..\scripts
python %SCRIPTS%\generate_blobs.py %base_name%
