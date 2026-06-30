# nav2_amcl — Test Procedure

- Node: `nav2_amcl::AmclNode`
- Lifecycle node (`--lifecycle`)

Make sure the three workspaces are fully built first (see `symros/SETUP.md`).

## configuration

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_amcl/ --node nav2_amcl::AmclNode --lifecycle --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_amcl/ --mode config
```

## construct test driver

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_amcl/ --node nav2_amcl::AmclNode --lifecycle --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_amcl/ --mode construct --test-id 3 \
&& ./rebuild_symros_exec.bash
```

## symbolic execution

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_amcl/ --node nav2_amcl::AmclNode --lifecycle --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_amcl/ --mode symex --test-id 3 --grater --no-build 2> /dev/null
```

## bug-guided input space exploration

```bash
python3 symros.py --project ../workspace/sym_ws/src/navigation2/nav2_amcl/ --node nav2_amcl::AmclNode --lifecycle --sym-ws ../workspace/sym_ws/ --conc-ws ../workspace/conc_ws/ --recover-ws ../workspace/chk_ws/ --result ./samples/nav2_amcl/ --mode negtest --test-id 3 --grater --no-build 2> /dev/null
```
