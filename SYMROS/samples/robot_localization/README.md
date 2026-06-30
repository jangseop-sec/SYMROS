# robot_localization — Test Procedure

- No `--node` / `--lifecycle`. A separate entry code is provided at
  `config/entry_code.cpp`.

Make sure the three workspaces are fully built first (see `symros/SETUP.md`).

## configuration

```bash
python3 symros.py --project ../workspace/sym_ws/src/robot_localization --sym-ws ../workspace/sym_ws --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/robot_localization/ --mode config
```

> Note: `config` does not build by itself. Replace the symbolic-executor
> package's `CMakeLists.txt` (under `workspace/(conc|sym|chk)_ws/symros_exec/`)
> with the dedicated `CMakeLists.txt.(sym|conc)`, then rebuild the config code
> and run.

```bash
./rebuild_symros_exec.bash
```

## construct test driver

```bash
python3 symros.py --project ../workspace/sym_ws/src/robot_localization --sym-ws ../workspace/sym_ws --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/robot_localization/ --mode construct --test-id 1 \
&& ./rebuild_symros_exec.bash
```

## symbolic execution

```bash
python3 symros.py --project ../workspace/sym_ws/src/robot_localization --sym-ws ../workspace/sym_ws --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/robot_localization/ --mode symex --test-id 1 --grater --no-build 2> /dev/null
```

## bug-guided input space exploration

```bash
python3 symros.py --project ../workspace/sym_ws/src/robot_localization --sym-ws ../workspace/sym_ws --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/robot_localization/ --mode negtest --test-id 1 --grater --no-build 2> /dev/null
```
