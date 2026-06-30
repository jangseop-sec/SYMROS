# nav2_behaviors — Test Procedure

- Node: `behavior_server::BehaviorServer`
- Lifecycle node (`--lifecycle`)

Make sure the three workspaces are fully built first (see `symros/SETUP.md`).

## configuration

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_behaviors/ --node behavior_server::BehaviorServer --lifecycle --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_behaviors --mode config
```

## construct test driver

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_behaviors/ --node behavior_server::BehaviorServer --lifecycle --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_behaviors --mode construct --test-id 1 \
&& ./rebuild_symros_exec.bash
```

## symbolic execution

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_behaviors/ --node behavior_server::BehaviorServer --lifecycle --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_behaviors --mode symex --test-id 1 --grater --no-build 2> /dev/null
```

## bug-guided input space exploration

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_behaviors/ --node behavior_server::BehaviorServer --lifecycle --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_behaviors --mode negtest --test-id 1 --grater --no-build 2> /dev/null
```
