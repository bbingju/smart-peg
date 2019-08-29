#include "mpc.h"
#include "event_source.h"

struct cmd_parser {
    mpc_parser_t* Decimal;
    mpc_parser_t* Hexadecimal;
    mpc_parser_t* Number;
    mpc_parser_t* Func;
    mpc_parser_t* Args;
    mpc_parser_t* Cmd;
    mpc_parser_t* Script;
};

static struct cmd_parser _p;

#if 0
/**
 * hex2int
 * take a hex string and convert it to a 32bit number (max 8 hex digits)
 */
static uint32_t hex2int(char *hex) {
    uint32_t val = 0;
    while (*hex) {
        // get current character then increment
        uint8_t byte = *hex++;
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        // shift 4 to make space for new digit, and add the 4 bits of the new digit
        val = (val << 4) | (byte & 0xF);
    }
    return val;
}
#endif

static uint32_t _value(mpc_ast_t *arg)
{
    if (!arg)
        return 0;

    if (strstr(arg->tag, "hexa")) {
        return (uint32_t) strtol(&arg->contents[2], NULL, 16);
    } else if (strstr(arg->tag, "decimal")) {
        return (uint32_t) atoi(arg->contents);
    } else
        return 0;
}

static int post_event_0(const char *funcname)
{
    int event = 0;
    struct peg_event_arg data = { 0 };

    if (strcmp(funcname, "led_clear") == 0) {
        event = PEG_EVENT_LED_CLEAR;
    } else if (strcmp(funcname, "led_draw") == 0) {
        event = PEG_EVENT_LED_DRAW;
    } else {
        return -1;
    }

    esp_event_post(PEG_EVENTS, event, NULL, 0, portMAX_DELAY);
    return 0;
}

static int post_event_1(const char *funcname, uint32_t a1)
{
    int event = 0;
    struct peg_event_arg data = { 0 };

    if (strcmp(funcname, "led_set_direct_draw") == 0) {
        event = PEG_EVENT_LED_SET_DIRECT_DRAW;
    } else {
        return -1;
    }

    data.argnum = 1;
    data.args[0] = a1;

    esp_event_post(PEG_EVENTS, event, &data, sizeof(data), portMAX_DELAY);

    return 0;
}

static int post_event_2(const char *funcname, uint32_t a1, uint32_t a2)
{
    return -1;
}

static int post_event_3(const char *funcname, uint32_t a1, uint32_t a2, uint32_t a3)
{
    int event = 0;
    struct peg_event_arg data = { 0 };

    if (strcmp(funcname, "led_set_pixel") == 0) {
        event = PEG_EVENT_LED_SET_PIXEL;
    } else {
        return -1;
    }

    data.argnum = 3;
    data.args[0] = a1;
    data.args[1] = a2;
    data.args[2] = a3;

    esp_event_post(PEG_EVENTS, event, &data, sizeof(data), portMAX_DELAY);

    return 0;
}

static int post_event_4(const char *funcname, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t a4)
{
    return -1;
}

static int post_event_5(const char *funcname, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t a4, uint32_t a5)
{
    int event = 0;
    struct peg_event_arg data = { 0 };

    if (strcmp(funcname, "led_fill_rect") == 0) {
        event = PEG_EVENT_LED_FILL_RECT;
    } else {
        return -1;
    }

    data.argnum = 5;
    data.args[0] = a1;
    data.args[1] = a2;
    data.args[2] = a3;
    data.args[3] = a4;
    data.args[4] = a5;

    esp_event_post(PEG_EVENTS, event, &data, sizeof(data), portMAX_DELAY);

    return 0;
}

int eval_commands(mpc_ast_t *root)
{
    if (!root)
        return -1;

    for (int i = 1; i < root->children_num - 1; i++) {

        mpc_ast_t *cmd = root->children[i];

        if (cmd->children_num == 3) { // no args
            mpc_ast_t *f = cmd->children[1];
            post_event_0(f->contents);
        }
        else if (cmd->children_num == 4) { // args exist
            mpc_ast_t *f = cmd->children[1];
            mpc_ast_t *args = cmd->children[2];
            int argnum = args->children_num / 2;

            if (argnum == 1) {
                post_event_1(f->contents, _value(args->children[1]));
            } else if (argnum == 2) {
                post_event_2(f->contents, _value(args->children[1]), _value(args->children[3]));
            } else if (argnum == 3) {
                post_event_3(f->contents, _value(args->children[1]), _value(args->children[3]),
                                  _value(args->children[5]));
            } else if (argnum == 4) {
                post_event_4(f->contents, _value(args->children[1]), _value(args->children[3]),
                                  _value(args->children[5]), _value(args->children[7]));
            } else if (argnum == 5) {
                post_event_5(f->contents, _value(args->children[1]), _value(args->children[3]),
                                  _value(args->children[5]), _value(args->children[7]), _value(args->children[9]));
            }
        }
    }

    return 0;
}

int cmd_parser_init()
{
    _p.Decimal     = mpc_new("decimal");
    _p.Hexadecimal = mpc_new("hexadecimal");
    _p.Number      = mpc_new("number");
    _p.Func        = mpc_new("func");
    _p.Args        = mpc_new("args");
    _p.Cmd         = mpc_new("cmd");
    _p.Script      = mpc_new("script");

    mpca_lang(MPCA_LANG_DEFAULT,
              " decimal     : /[0-9]+/ ;                  \n"
              " hexadecimal : /0x[0-9a-fA-F]+/ ;          \n"
              " number      : <hexadecimal> | <decimal> ; \n"
              " func        : /[a-zA-Z][a-zA-Z0-9_]*/ ;   \n"
              " args        : (',' <number>)* ;           \n"
              " cmd         : '(' <func><args> ')' ;      \n"
              " script      : /^/ <cmd>* /$/ ;            \n",
              _p.Decimal, _p.Hexadecimal, _p.Number, _p.Func, _p.Args, _p.Cmd, _p.Script, NULL);

    return 0;
}

void cmd_parser_deinit()
{
    mpc_cleanup(7, _p.Decimal, _p.Hexadecimal, _p.Number, _p.Func, _p.Args, _p.Cmd, _p.Script);
}

int cmd_parser_eval(const char *str, int n)
{
    mpc_result_t r;

    if (mpc_nparse("test", str, n, _p.Script, &r)) {
        eval_commands(r.output);
        mpc_ast_delete(r.output);

    } else {
        mpc_err_print(r.error);
        mpc_err_delete(r.error);
    }

    return 0;
}
