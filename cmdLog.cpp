/*

Module:	cmdLog.cpp

Function:
        Process the "log" command

Copyright and License:
        See accompanying LICENSE file for copyright and license information.

Author:
        Pranau R, MCCI Corporation   May 2023

*/

#include "Catena4802_cmd.h"

#include <Catena_Log.h>

using namespace McciCatena;

/*

Name:   ::cmdLog()

Function:
        Command dispatcher for "log" command.

Definition:
        McciCatena::cCommandStream::CommandFn cmdLog;

        McciCatena::cCommandStream::CommandStatus cmdLog(
            cCommandStream *pThis,
            void *pContext,
            int argc,
            char **argv
            );

Description:
        The "log" command has the following syntax:

        log
            Display the current log mask.

        log {number}
            Set the log mask to {number}

Returns:
        cCommandStream::CommandStatus::kSuccess if successful.
        Some other value for failure.

*/

// argv[0] is "log"
// argv[1] is new log flag mask; if omitted, mask is printed
cCommandStream::CommandStatus cmdLog(
    cCommandStream *pThis,
    void *pContext,
    int argc,
    char **argv
    )
    {
    if (argc > 2)
        return cCommandStream::CommandStatus::kInvalidParameter;

    if (argc == 1)
        {
        pThis->printf("log flags: %#x\n", gLog.getFlags());
        return cCommandStream::CommandStatus::kSuccess;
        }
    else
        {
        cCommandStream::CommandStatus status;
        uint32_t newFlags;

        // get arg 1 as newFlags; default is irrelevant
        status = cCommandStream::getuint32(argc, argv, 1, /*radix*/ 0, newFlags, /* default */ 0);
        if (status == cCommandStream::CommandStatus::kSuccess)
            {
            cLog::DebugFlags const oldFlags =
                gLog.setFlags(cLog::DebugFlags(newFlags));

            pThis->printf("log flags: %#x -> %#x\n", oldFlags, newFlags);
            }
        return status;
        }
    }