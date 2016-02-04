#include "hlcmd.h"
#include "llcmd.h"
#include <stddef.h>
#include <string.h>

namespace hlcmd {
    static size_t min(size_t x, size_t y) {
        return x < y ? x : y;
    }

    void process(const void *data, size_t len) {
        // TODO: Compile hl commands into ll commands.
        if(!llcmd::idle()) {
            // TODO: Some commands (e.g. kick) should not be aborted even if
            // a new order comes in. Figure something out. (Maybe even keep
            // kicking while moving?)
            llcmd::finish(false);
        }
        memcpy(llcmd::cmds, data, min(len, llcmd::cmd_cap));
        llcmd::cmd_len = len;
        llcmd::cmd_at = 0;
        llcmd::start();
    }
}
