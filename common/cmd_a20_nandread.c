#include <common.h>
#include <command.h>
#include <nand.h>

static int do_a20_nandread(cmd_tbl_t *cmdtp, int flag, int argc,
		char * const argv[])
{
	if (argc != 4) {
		printf("usage: a20_nandread <address> <offset> <bytes>\n");
		return 1;
	}

	uint32_t dst = simple_strtoul(argv[1], NULL, 16);
	uint32_t src = simple_strtoul(argv[2], NULL, 16);
	uint32_t cnt = simple_strtoul(argv[3], NULL, 16);
	printf("Reading 0x%08X bytes from NAND @ 0x%08X to MEM @ 0x%08X...\n",
	       cnt, src, dst);
	nand_spl_load_image(src, cnt, (void *)dst);
	printf("\n");
	return 0;
}

U_BOOT_CMD(a20_nandread, CONFIG_SYS_MAXARGS, 3,
	   do_a20_nandread, "a20_nandread", "[offset size bytes]\n" "   "
);
