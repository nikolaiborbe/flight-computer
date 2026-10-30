#include "Threads/Entry/Logging.hpp"
#include "Context.hpp"

namespace Thread::Entry
{
    void Logging(ULONG thread_input)
    {
        (void)thread_input;
        System::file_logger->run();
    }
}