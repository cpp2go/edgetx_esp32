#!/usr/bin/env python
# Scratch helper: regenerate radio/src/storage/yaml/yaml_datastructs_openx1.cpp
# using libclang + the real OPENX1 ESP-IDF compile flags from compile_commands.json.
import json, os, shlex, subprocess, sys

ROOT = r"E:\develop\dev\edgetx_esp32"
RADIO_SRC = os.path.join(ROOT, "radio", "src")
BUILD = os.path.join(RADIO_SRC, "targets", "openx1", "esp32_build", "build")
CC = os.path.join(BUILD, "compile_commands.json")
OUT = os.path.join(RADIO_SRC, "storage", "yaml", "yaml_datastructs_openx1.regenerated.cpp")
GEN = os.path.join(ROOT, "radio", "util", "generate_yaml.py")
TMPL = os.path.join(ROOT, "radio", "util", "yaml_parser.tmpl")

def load_flags():
    entries = json.load(open(CC, encoding="utf-8"))
    entry = None
    for e in entries:
        f = e["file"].replace("\\", "/")
        if f.endswith("storage/sdcard_common.cpp"):
            entry = e
            break
    if entry is None:
        sys.exit("sdcard_common.cpp entry not found")
    toks = shlex.split(entry["command"], posix=True)
    # expand @response files
    exp = []
    for t in toks:
        if t.startswith("@"):
            p = t[1:]
            if not os.path.isabs(p):
                p = os.path.join(entry["directory"], p)
            try:
                with open(p, encoding="utf-8", errors="replace") as fh:
                    exp.extend(shlex.split(fh.read(), posix=True))
            except OSError as ex:
                print("WARN cannot read response file", p, ex)
        else:
            exp.append(t)
    return exp

def is_obj_or_src(s):
    low = s.lower()
    return low.endswith((".c", ".cpp", ".cxx", ".cc", ".s", ".S", ".obj", ".o"))

def filter_flags(toks):
    DROP_PREFIX = ("-march=", "-mcpu=", "-mabi=", "-mtune=", "-mlongcalls",
                   "-mcmodel=", "-msmall-data-limit=", "-misa-spec=", "-mrelax",
                   "-mno-relax", "-mno-", "-m", "-fmacro-prefix-map=",
                   "-fno-jump-tables", "-fno-tree-switch-conversion",
                   "-fstrict-volatile-bitfields", "-fno-shrink-wrap",
                   "-fuse-cxa-atexit", "-gdwarf", "-ggdb", "-g", "-Werror",
                   "-Winvalid-pch", "-fdiagnostics-color")
    out = []
    i = 0
    n = len(toks)
    while i < n:
        t = toks[i]
        low = t.lower()
        # skip compiler binary
        if i == 0 and low.endswith(".exe"):
            i += 1
            continue
        if t == "-include":           # drop force-include (ff_compat.h) - not needed for YAML TU
            i += 2
            continue
        if t in ("-c", "-o", "-MD", "-MMD", "-MP", "-MF", "-MT", "-MQ"):
            i += 1
            if t in ("-o", "-MF", "-MT", "-MQ"):
                i += 1
            continue
        if is_obj_or_src(t):
            i += 1
            continue
        if t.startswith("@"):
            i += 1
            continue
        if any(t.startswith(p) for p in DROP_PREFIX):
            i += 1
            continue
        if t.startswith("-Wno-"):     # keep suppressions (harmless once unknown-warning-option disabled)
            out.append(t)
            i += 1
            continue
        if t.startswith("-W"):        # generator bails on ANY warning -> drop enabling -W
            i += 1
            continue
        if (t.startswith("-D") or t.startswith("-U") or t.startswith("-I")
                or t.startswith("-isystem") or t.startswith("-idirafter")
                or t.startswith("-std=") or t.startswith("-nostdinc")
                or t.startswith("-fno-exceptions") or t.startswith("-fno-rtti")
                or t in ("-x", "-ansi", "-pedantic")):
            out.append(t)
        i += 1
    # keep only the LAST -std= (matches real compiler behaviour; overrides generator's c++11)
    st = [x for x in out if x.startswith("-std=")]
    if st:
        out = [x for x in out if not x.startswith("-std=")] + [st[-1]]
    return out

def main():
    toks = load_flags()
    fl = filter_flags(toks)
    # ensure generator macro
    if "-DYAML_GENERATOR" not in fl:
        fl.append("-DYAML_GENERATOR")
    # generator prepends its own -x c++ -std=c++11; remove conflicting -std from our list to be safe
    fl = [t for t in fl if not t.startswith("-std=")]
    # toolchain include dirs + 32-bit target semantics (long==4 etc.)
    PC = "C:/Espressif/tools/riscv32-esp-elf/esp-15.2.0_20251204/riscv32-esp-elf"
    extra = [
        "--target=riscv32-esp-elf",
        "-isystem", PC + "/lib/gcc/riscv32-esp-elf/15.2.0/include",
        "-isystem", PC + "/picolibc/include",
        "-isystem", PC + "/picolibc/riscv32-esp-elf/sys-include",
        "-isystem", PC + "/picolibc/riscv32-esp-elf/include/c++/15.2.0",
        "-isystem", PC + "/picolibc/riscv32-esp-elf/include/c++/15.2.0/riscv32-esp-elf",
        "-isystem", PC + "/picolibc/riscv32-esp-elf/include/c++/15.2.0/backward",
    ]
    fl = extra + fl
    # suppress clang-default warnings that would make the generator bail
    fl = fl + ["-Wno-extern-c-compat", "-Wno-unknown-warning-option"]
    args = [sys.executable, GEN, "myeeprom.h", TMPL, "RadioData,ModelData,PartialModel"] + fl
    print("driver python:", sys.executable)
    print("total filtered flags:", len(fl))
    env = dict(os.environ)
    # make libclang.dll findable by find_clang.py (PATH scan)
    sp = os.path.join(ROOT, ".venv", "Lib", "site-packages", "clang", "native")
    if os.path.isdir(sp):
        env["PATH"] = sp + os.pathsep + env.get("PATH", "")
    print("run in", RADIO_SRC)
    with open(OUT, "w", encoding="utf-8") as fh:
        r = subprocess.run(args, cwd=RADIO_SRC, env=env, stdout=fh,
                           stderr=subprocess.PIPE)
    sys.stderr.write(r.stderr.decode("utf-8", "replace"))
    print("exit:", r.returncode)
    print("output:", OUT, os.path.getsize(OUT) if os.path.exists(OUT) else "MISSING")

if __name__ == "__main__":
    main()
