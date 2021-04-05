//***************************************************************************
// examples/helloxx/helloxx_main.cxx
//
//   Copyright (C) 2009, 2011-2013, 2017 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <gnutt@nuttx.org>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name NuttX nor the names of its contributors may be
//    used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//***************************************************************************

//***************************************************************************
// Included Files
//***************************************************************************

#include <nuttx/config.h>

#include <cstdio>
#include <debug.h>

#include <nuttx/init.h>

#include "platform/cxxinitialize.h"

//***************************************************************************
// Definitions
//***************************************************************************
// Configuration ************************************************************
// C++ initialization requires CXX initializer support

#if !defined(CONFIG_HAVE_CXX) || !defined(CONFIG_HAVE_CXXINITIALIZE)
#  undef CONFIG_EXAMPLES_HELLOXX_CXXINITIALIZE
#endif

// Debug ********************************************************************
// Non-standard debug that may be enabled just for testing the constructors

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_CXX
#endif

#ifdef CONFIG_DEBUG_CXX
#  define cxxinfo     _info
#else
#  define cxxinfo(x...)
#endif

//***************************************************************************
// Private Classes
//***************************************************************************

class CHelloWorld
{
  public:
    CHelloWorld(void) : mSecret(42)
    {
      cxxinfo("Constructor: mSecret=%d\n", mSecret);
    }

    ~CHelloWorld(void)
    {
      cxxinfo("Destructor\n");
    }

    bool HelloWorld(void)
    {
        cxxinfo("HelloWorld: mSecret=%d\n", mSecret);

        if (mSecret != 42)
          {
            printf("CHelloWorld::HelloWorld: CONSTRUCTION FAILED!\n");
            return false;
          }
        else
          {
            printf("CHelloWorld::HelloWorld: Hello, World!!\n");
            return true;
          }
    }

  private:
    int mSecret;
};

//***************************************************************************
// Private Data
//***************************************************************************

// Define a statically constructed CHellowWorld instance if C++ static
// initializers are supported by the platform

#ifdef CONFIG_HAVE_CXXINITIALIZE
static CHelloWorld g_HelloWorld;
#endif

//***************************************************************************
// Public Functions
//***************************************************************************

/****************************************************************************
 * Name: helloxx_main
 ****************************************************************************/

extern "C"
{
  int main(int argc, FAR char *argv[])
 {
    // If C++ initialization for static constructors is supported, then do
    // that first

#ifdef CONFIG_EXAMPLES_HELLOXX_CXXINITIALIZE
    up_cxxinitialize();
#endif

    // Exercise an explictly instantiated C++ object

    CHelloWorld *pHelloWorld = new CHelloWorld;
    printf("helloxx_main: Saying hello from the dynamically constructed instance\n");
    pHelloWorld->HelloWorld();

    // Exercise an C++ object instantiated on the stack

#ifndef CONFIG_EXAMPLES_HELLOXX_NOSTACKCONST
    CHelloWorld HelloWorld;

    printf("helloxx_main: Saying hello from the instance constructed on the stack\n");
    HelloWorld.HelloWorld();
#endif

    // Exercise an statically constructed C++ object

#ifdef CONFIG_HAVE_CXXINITIALIZE
    printf("helloxx_main: Saying hello from the statically constructed instance\n");
    g_HelloWorld.HelloWorld();
#endif

    delete pHelloWorld;
    return 0;
  }
}
