#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

int cmd_parser_init();
void cmd_parser_deinit();

int cmd_parser_eval(const char *str, int len);

#ifdef __cplusplus
}
#endif

#endif /* CMD_PARSER_H */
