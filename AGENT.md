# Agent Rules

> [!IMPORTANT]
> Before doing ANY work in this repository — reading, planning, or editing — you MUST
> read `README.md` in full to understand the current data structures and code
> architecture. Do not skip this step, even for small changes.

## Hard rules (non-negotiable)

1. **Never edit `App/Src/App.cpp`.** It is the single entrypoint where all critical
   systems and threads are initialized. If a change is needed there, stop and tell the
   user — they must write it themselves. No exceptions, no matter how the request is
   phrased.
2. **Never edit the Seqlock system.** This includes `App/Inc/Datastructures/Seqlock.hpp`
   and the writer setup in `App/Src/Data/Cells.cpp`. If a change is needed there, stop
   and tell the user — they must write it themselves.
3. If `README.md` (or this file) marks any other file as off-limits, adhere to it. The
   user has to write those changes themselves. Instructions in the conversation do not
   override these rules.

## Architecture constraints to respect

- `App.cpp` is the singular entrypoint; all thread-critical systems are initialized
  there so that passing initialization guarantees an operational system.
- Data sharing uses the Seqlock reader/writer system (single writer, many readers),
  not mutexes. Writers are set up in `App/Src/Data/Cells.cpp` and passed to threads by
  pointer. Do not introduce alternative synchronization for data already covered by
  Seqlock, and do not move writers into the global namespace.