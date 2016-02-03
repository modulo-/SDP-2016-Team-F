#include "hlcmd.h"
#include "llcmd.h"
#include <stddef.h>
#include <stdlib.h>

namespace hlcmd {
    void process(void *data, size_t len) {
        // TODO: Compile hl commands into ll commands.
        if(!llcmd::idle()) {
            // TODO: Some commands (e.g. kick) should not be aborted even if
            // a new order comes in. Figure something out. (Maybe even keep
            // kicking while moving?)
            llcmd::finish(false);
        }
        free(llcmd::cmds);
        llcmd::cmds = (uint8_t *)data;
        llcmd::cmd_len = len;
        llcmd::cmd_at = 0;
        llcmd::start();
    }
}
