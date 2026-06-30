for n in range(1, 65):
    checks = "\n".join(
        [f"        if (*(s1 + {i}) != *(s2 + {i})) \\\n"
         f"            (ret) &= false; \\" for i in range(n)]
    )

    print(f"#define STRCMP_{n}(s1, s2, ret) \\\n"
          f"    do {{ \\\n"
          f"{checks}\n"
          f"    }} while (0)\n")