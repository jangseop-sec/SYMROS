# nav2_controller — Test Procedure

- No `--node` / `--lifecycle`. A separate entry code is provided at
  `config/entry_code.cpp`.

Make sure the three workspaces are fully built first (see `symros/SETUP.md`).

## configuration

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_controller/ --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_controller/ --mode config \
&& ./rebuild_symros_exec.bash
```

## construct test driver

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_controller/ --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_controller/ --mode construct --test-id 1 \
&& ./rebuild_symros_exec.bash
```

## symbolic execution

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_controller/ --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_controller/ --mode symex --test-id 1 --grater --no-build 2> symros.log
```

## bug-guided input space exploration

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_controller/ --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_controller/ --mode negtest --test-id 1 --grater --no-build 2> symros.log
```
