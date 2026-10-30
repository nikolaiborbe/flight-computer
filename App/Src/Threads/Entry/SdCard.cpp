#include "Threads/Entry/SdCard.hpp"
#include "Context.hpp"

namespace Thread::Entry
{
    void SdCard(ULONG thread_input)
    {
        (void)thread_input;
        System::storage_manager->run();
    }
}
