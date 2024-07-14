# yaxgbc

another Game Boy (DMG) and Game Boy Color (CGB) emulator. mostly works?

for the handful of programs i want to usably support things seem basically workable:
* Link's Awakening
* Link's Awakening DX
* Is That A Demo In Your Pocket?
* Oracle of Ages/Seasons
  - might still be cgb-mode priority issues: item spites do not display in the window
* Pokemon Blue/Red/Yellow
  - some timing issue remains: when Pidgey (or a few other pokemon) comes onscreen the voice plays a few seconds too long
* Pokemon Gold/Silver/Crystal
  - start, seem to kind of work, but RTC is not emulated at all so who knows how much that holds up
* Super Mario Land 2: 6 Golden Coins
  - works well enough? haven't played much.

if you use it let me know! i'm not sure i expect it to be usable by anyone, but a clone and `cargo build --release` will work.

# known issues

* Perfect Dark barely works, not sure why. probably mishandling an interrupt somehow.
* APU simulation can produce awful high frequencies sometimes. probably need a low-pass filter on the output
* some games flip between dark/light pixels to produce a translucency effect
  (notably, Prehistorik Man, or the crane game in Link's Awakening). this is
  not emulated, so it's just a kind of annoying flicker.

# why?

this started as an excuse to use
[`yaxpeax-sm83`](https://github.com/iximeow/yaxpeax-sm83) in a way where both
decoder performance is important *and* throughput of the whole program is
important. this in turn resulted in [mostly rewriting
`yaxpeax-sm83`](https://github.com/iximeow/yaxpeax-sm83/commit/819e8a30d20c28398a00976a9925e9e741950bee)
in a way that works here. it will probably be a model for future yaxpeax
disassembler interfaces - similar seems prudent for x86, arm, powerpc, mips,
etc: anywhere where the overhead of dispatching on common enums is comparable
to the work to decode and execute in the first place.

the "specialize on a pile of callbacks" approach like this, instead, fuses
decoding into an execution loop more like what ad-hoc disassemblers often look
like. it was easily an integer multiple improvement for yaxgbc. libraries can
be suitable in these contexts!

# what else?

no link cable emulation. wouldn't hurt to add, never needed it.

no IR emulation. same.

no RTC emulation. same.

debug interfaces are somewhat poor.

yaxgbc is about 2x slower than mgba in terms of top speed. i'm not sure why, i'd like to fix that.

i'd rather like to cross-compile dmg/cgb roms to native executables eventually,
so yaxgbc may grow such an "AOT" mode one day.
