/**
 * @file buzzer_sheet.c
 * @brief 蜂鸣器乐谱播放
 *
 * 乐谱以按键字符书写，复用 buzzer.c 内的键位绑定表(BUZZER_KEY_NOTE_TABLE)：
 * 谱子里写 's' 即等于 NOTE_C5，写 'f' 即等于 NOTE_E5，与 buzzer piano 一致。
 *
 * 乐谱格式(score 字符串)：
 *   - 已绑定的按键字符   : 播放一个单位时长的该音符
 *   - '='                : 休止一个单位时长
 *   - '-'                : 把前一个音符再延长一个单位时长(延音线，同音)
 *   - '_'                : 连音线，使前后两个音无缝相连(不插分音间隙)
 *   - '[音1音2...]'       : 和弦，括号内多个音快速轮播近似(最多 BUZZER_SHEET_CHORD_MAX 个)
 *   - ' '、'|' 或其他未绑定字符 : 忽略且不占时长，仅用于阅读时分隔、便于断句
 *
 * shell 用法：
 *   sheet            列出所有乐谱
 *   sheet <name>     播放指定乐谱(播放中按 ESC 可中断)
 */

#include "buzzer.h"
#include "cmsis_os2.h"
#include "shell.h"
#include "log.h"
#include <string.h>

#if !BUZZER_ACTIVE_MODE

#define BPM_TO_MS(bpm)  ((60000 + (bpm)/2) / (bpm))

/** 相邻两个音符之间插入的短休止(ms)，用于分音，避免连成一片 */
#define BUZZER_SHEET_GAP_MS 15U

/** 退出播放的按键(默认 ESC) */
#define BUZZER_SHEET_ABORT_CHAR '\x1b'

/** 一个和弦最多包含的音符数 */
#define BUZZER_SHEET_CHORD_MAX 4U

/** 和弦轮播时每个音的时长(ms)，越小越"融合"，越大越像分解琶音 */
#define BUZZER_SHEET_CHORD_TONE_MS 100U

/** 一条乐谱 */
typedef struct {
    const char *name; /* 乐谱名，供 shell 指定播放 */
    const char *score; /* 乐谱内容，字符含义见文件头说明 */
    uint16_t unit_ms; /* 一个单位(一拍)的时长(ms) */
} Buzzer_Sheet_t;

/** 乐谱表：新增乐谱只需在此追加一行 */
static const Buzzer_Sheet_t buzzer_sheets[] = {
    /* 音阶：C4B5 依次上行，用于试音/找谐振点 */
    {"scale", "tyuiopasdfghjk", 200},
    /* 小星星：C C G G A A G- / F F E E D D C- */
    {"twinkle", "ttooppo-,iiuuyyt-", 260},
    /* 欢乐颂开头：E E F G / G F E D / C C D E / E D D- */
    {"ode", "uuio,oiuy,ttyu,uyy-", 260},
    /* 和弦示例：C - F - G - C 四个和弦(快速琶音近似) */
    {"chord", "[sfh]=[ips]=[oad]=[sfh]-", 320},

    /* 三叶的主题曲 */
    {"triple_leaf",
     "s_d_[if]h[og]f[ud]s[if]___,"
     "o_p_[is]d[of]a[us]o[ip]___,"
     "s_d_[if]h[og]f[ud]s[if]___,"
     "o_p_[is]d[of]a[us]o[ip]___,"
     "s_d_[if]h[og]f[ud]s[if]___,"
     "o_p_[is]d[of]a[us]o[ip]___,"
     "[qs]_h_s_h_s-s_h_s_h_s,"
     "[es]_h_s_h_s-ws_d_s_f_s,"
     "[qs]_h_s_h_s-s_h_s_h_s,"
     "[es]_h_s_h_s-w_k_k_h_j_f_h_a_"
     "[qs]_h_s_h_s-s_h_s_h_s,"
     "[es]_h_s_h_s-ws_d_s_f_s,"
     "[qs]_d_s_f_s-s_d_s_f_s,"
     "[qs]_d_s_f_s-,"
     "[sht]osodo[sf]o,"
     "[esh]pspdp[sg]_f_d_s_,"
     "qioipi[os]i,"
     "[sft]oso[wd]oso,"
     "[sht]osodo[sf]o,"
     "[esh]pspdp[sg]_f_d_s_,"
     "qioipi[os]i,"
     "[sft]oso[adw],"
     "s_d_[if]h[og]f[ud]s[if]___,"
     "o_p_[is]d[of]a[us]o[ip]___,"
     "s_d_[if]h[og]f[ud]s[if]___,"
     "o_p_[si]a[od]s[Pf]d[Ph]g[Pj]h[Pl]J,"
     "[dpj]_ h_ j_ [dpz]_ j_ h_ [dpj]_ h_ j_ [dpz]_ j_ h_,"
     "[pSj]_ h_ j_ [pSL]_ j_ h_ [pSj]_ h_ j_ [pSL]_ j_ h_,"
     "[dpj]_ h_ j_ [dpz]_ j_ h_ [dpj]_ h_ j_ [dpz]_ j_ h_,"
     "[pSj]_ h_ j_ L_ h_ j_ z_ h_ j_ [Lxb]---,"
     "[qs]_h_s_h_s-s_h_s_h_s,"
     "[es]_h_s_h_s-ws_d_s_f_s,"
     "[qs]_h_s_h_s-s_h_s_h_s,"
     "[es]_h_s_h_s-w_l_k_h_j_f_h_a_,"
     "[qs]_d_s_f_s-s_d_s_f_s,"
     "[qs]_d_s_f_s-,"
     "[qs]_h_s_f_s-,"
     "t_y_u_o_[4p]u[5o]y[6u]t[8y]_o_u---,"
     "[4w]t[5r]y[6t]u[8y]o---,"
     "t_y_u_o_[4p]u[5o]y[6u]t[8y]_o_u---,"
     "[4w]t[5r]y[6t]u[8y]t---,"
     "[4p]_o_[qs]_a_[5p]_o_[wu]_o_[3y]_u_[0t]_y_[6e]_t_[ey]_u_,"
     "[4p]_o_[qs]_a_[5p]_o_[wu]_o_[3p]_s_[0d]_f_[6d]_s_[ep],"
     "[4p]_o_[qs]_a_[5p]_o_[wu]_o_[3y]_u_[0t]_y_[6e]_t_[ey]_u_,"
     "[4s]_q_o_p_q_u_o_q_y_u_q_t_y_q_e,"
     "[4s]_q_o_p_q_u_o_q_y_u_q_t_y_q_e,"
     "[3s]_0_o_p_0_u_o_0_y_u_0_t_y_0_e,"
     "[3s]_0_o_p_0_u_o_0_y_u_0_t_y_0_e,"
     "[4s]_q_o_p_q_u_o_q_y_u_q_t_y_q_e,"
     "[4s]_q_o_d_q_s_f_q_d_h_q_f_j_q_h_z",
     500},
    /* 蒲公英的约定 */
    {"TDP",
     "  u = o = s = f  = fqet g[qd] ry = [0d]w[rh]a[0s]et   = [9f][qg][eh]ss  9w[rd]w[8wtf] = [8wtu]=[uE] o   "
     "[sQ]ffqet g[qd]ry =  [0d]w[rh]a[0s]et  [9f][g0][wh][0s]9w[rd]w][8wtf] =  [tips]= [yi]t tuio  [80wo]=[80w]=[80wo] "
     "[70]p[0wra]s   [etus]=[ets]=[ewtu] adp [qet]o[qet]d[wry]s[wry] [w8f][wtf] ==[wtu]s[wry] [w8f][wtf ]= "
     "[wtu]=sd[wtuf][wf] = [ryp]fgf [ruO]dfs|[etus] [ets]=[wtup]asd[9qt]s[9qt]p[9qt]f[9qt]d                    [wti] = "
     "[wry] = u o s f fqet[qg][qf] ryr [0d]wh[rwa][0s] ete=  [9f][qg][eh][qs] 9w[rd]w[6f] 8wr[80ru] o [80rs]           "
     "        fqet[qg][qd] ryr [0d]w[rh][wa][0s] ere                               [9f][qg][eh][qs]9w                  "
     "                                        [rd] w [0wts] = [0wts] = [0wta] = [0wts] = [0etd] =[0ets] =[0eta] = "
     "[0ets] =[eyfQ] = [eydQ] = [eysQ] = [eydQ] = [qtfW] = [qtdW] = [gwry] = [wryd]=[0wtd] = [0wts] = [0wta] = [0wts] "
     "= [0etd] = [0ets] = [0eta] = [0ets] =[eyfQ] = [eydQ] = [eysQ] = [eydQ] = [qtfW] = [qtdW] = [gwry] = "
     "[wryd]=[0wts] [0ws] [0w]uio [i08w]o [i08w]o[70][70]pa[8wr]o[etd]s[es] = [wt]a[wtd]p "
     "=[qetu]o[qetu]d[wry]s[uf][uf][8w]t =[swtu]  [wtuf] [uf] [ryp]f [ryp] gfg [ruO][rudO] fs[etu] [etu] "
     "[wtup]asd[9qt]s [9qt]p [9qt]f [9qt]d[wti] =[wryu]os fqet[eg][qf] ryr [0d]w[rh][wa][0s] ete "
     "[9f][qg][eh][qs][9s]w[rd]w[8f]0wr =[80ru]p[80rs]f qet[eg][qd] ryr [0d]w[rh][wa][0s] ere [9f][qg][qh][qs][9s] "
     "w[rd]w[0wts] [0ws] [80uE]os[80uf][9qt]g[9qeh]s[9wr] [9wrd] [0wts] [8wti] [8wtu]",
     429},

     {"sunday",
        "= =_o_s_a_o_t_ | ~ o_ y u_~u_ | ~_ =_o_s_a_o_t_ | ~ o_y u_ i_u_ | ~"
        "=_o_d_s_a_s_ | ~ d_f s_~_p_ | ~ a_s o_~_t_ |  o_i_u_y t_~ | r_~ = |"
        "=__s__s__a__s_=__a__s__a__s__ =_d__~ |"
        "=__a__a__p__a_=__p__a__p__a_=_s_ |"
        "= p_s_f_d_s_f_ | ~ =_f_f_d__s__ |"
        "=__s__s__a__s_=__a__s__a__s_=_d__~ |"
        "=__a__a__p__a_=__p__a__p__a_=_s_ |"
        "= p_s_f_d_s_s_ | ~"
        "~ p_a_s_s_s_s_p p_a_ | s_s_s_d_d o_p_ |"
        "a_a_a_a_o o_p_ | a_a_a_s_s_s__ s_d_ |"
        "f_f_f_p_s s_d_ | f_f_j_f__d__ s s_d_ |"
        "f_f_f_f s_ f_d_ | ~"
        "~ =_o_g_f_ | ~g_f d_~ | s d_f s_~ | p s_h s_ f_f_ | ~"
        "~ =_o_g_f_ | ~g_f d_~ | s d_f j_~ | h a_s d_~~s_~~",
        429},

        {"star",
        "u u u u u u u u u u u u u u u [pu] u u [au] u u u u u [pu] [au] [su] [au] [pu] [au] [su] u u [au] u u u u u [pu] u [fu] [su] u u [au] u u u u u [pu] [fu] [su] [au] [pu] [au] [su] u u [au] u u u u u [pu] [fu] [su] [au] [pu] [au] [su] u u [du] u u [fu] u u "
        "[pu]a[su]a[pu]s[au]p[ou]p[au]o[au]p[ou]p[au]o[pu]s[au]d[su]p[au]o[pu]s[au]d[su]d[su]a[pu]s[au]p[ou]p[au]o[au]p[ou]f[au]o "
        "p u a u s u a u p u s u a u p u o u p u a u o u a u p u o u f u a u o u p u s u a u d u s u p u a u o u p u s u a u d "
        "f u f u f u f u f u f u [fu] [fu] [fu] [fu] [fu] [fu] [piu] f [piu] f [aou] f [aou] f [spu] f [spu] f [dou] f [dou] f a "
        "[piu]fff f[piu]fff f[aou]fff f[aou]fff f[spu]fff f[spu]fff f[dou]fff f[dou]fff f "
        "[pi] f [pi] f [ao] f [ao] f [sp] f [sp] f [do] f [do] f a [pi] f p [pi] f p [ao] f a [ao] f a [sp] f s [sp] f s [do] f d [do] f a [fu] f f f f f",
        429},

        {"super","fff=sfh=o|=u=pa=Poif|hjghfsda=s|=o=u=pa=Pp|ofhjghfsda|=hGgDd=sss|==hGgDfOps|=s=psd=hGg|Df=sss==hG|gDf=Ops=ps|d=h=g=f==s|ss=sdfspo=|sss=sdf==s|ss=sdfspo=|fff=sfh=o=|so=u=pa=Pp|ofhjghfsda|=s=o=u=pa=|Ppofhjghf|sda=fsoOpg|gp=ajjjhg|fspo=fsopg|",
        BPM_TO_MS(300)},
};

#define BUZZER_SHEET_COUNT (sizeof(buzzer_sheets) / sizeof(buzzer_sheets[0]))

/**
 * @brief 检查是否按下了中断键，并清空输入缓冲
 * @return true 按下了退出键(ESC)
 */
static bool Buzzer_Sheet_Aborted(void) {
    extern Shell shell;
    char ch;
    bool aborted = false;

    while (shell.read(&ch, 1)) {
        if (ch == BUZZER_SHEET_ABORT_CHAR) aborted = true;
    }
    return aborted;
}

/**
 * @brief 播放一个单位的音响(休止/单音/和弦)
 * @param freqs 音符频率数组
 * @param count 音符个数，0 表示休止
 * @param unit_ms 持续时长(ms)
 * @note  和弦用快速轮播近似，每个音停留 BUZZER_SHEET_CHORD_TONE_MS
 */
static void Buzzer_Sheet_PlayUnit(const uint16_t *freqs, uint32_t count, uint32_t unit_ms) {
    if (count == 0) { /* 休止 */
        osDelay(unit_ms);
        return;
    }

    if (count == 1) { /* 单音 */
        Buzzer_On(freqs[0]);
        osDelay(unit_ms);
        return;
    }

    /* 和弦：在 unit_ms 内按顺序快速轮播各音，听感接近持续和弦 */
    uint32_t elapsed = 0;
    uint32_t idx = 0;
    while (elapsed < unit_ms) {
        uint32_t step = BUZZER_SHEET_CHORD_TONE_MS;
        if (elapsed + step > unit_ms) step = unit_ms - elapsed;

        Buzzer_On(freqs[idx % count]);
        osDelay(step);

        elapsed += step;
        idx++;
    }
}

/**
 * @brief 阻塞播放一条乐谱
 * @param sheet 乐谱指针
 * @note  播放中按 ESC 可中断
 */
static void Buzzer_Sheet_Play(const Buzzer_Sheet_t *sheet) {
    const char *p = sheet->score;
    uint16_t cur_freqs[BUZZER_SHEET_CHORD_MAX]; /* 当前发声音(1个=单音, 多个=和弦) */
    uint32_t cur_count = 0; /* 0 表示当前无声 */
    bool legato = false; /* 下一个音是否与当前音无缝相连(连音线标记) */

    logPrintln("Playing sheet: %s (ESC to abort)", sheet->name);

    while (*p != '\0') {
        if (Buzzer_Sheet_Aborted()) {
            logPrintln("Sheet aborted");
            break;
        }

        char ch = *p;

        if (ch == '-' && cur_count > 0) { /* 延音线：当前音再延长一个单位 */
            Buzzer_Sheet_PlayUnit(cur_freqs, cur_count, sheet->unit_ms);
            p++;
            continue;
        }

        if (ch == '_') { /* 连音线：不发音、不占时长，仅标记前后两音无缝相连 */
            legato = true;
            p++;
            continue;
        }

        if (cur_count > 0) { /* 先给上一个音收尾；连音时不插分音间隙 */
            Buzzer_Off();
            cur_count = 0;
            if (!legato) osDelay(BUZZER_SHEET_GAP_MS);
            legato = false;
        }

        if (ch == '=') { /* 休止一个单位 */
            Buzzer_Sheet_PlayUnit(NULL, 0, sheet->unit_ms);
            legato = false;
            p++;
            continue;
        }

        if (ch == '[') { /* 和弦：[音1音2...]，最多 BUZZER_SHEET_CHORD_MAX 个音 */
            const char *q = p + 1;
            uint32_t n = 0;
            while (*q != '\0' && *q != ']' && n < BUZZER_SHEET_CHORD_MAX) {
                uint16_t freq_hz = 0;
                if (Buzzer_KeyToFreq(*q, &freq_hz)) cur_freqs[n++] = freq_hz;
                q++;
            }
            if (*q == ']') q++; /* 跳过结尾 ']'，缺失时直接结束本组 */

            if (n > 0) {
                Buzzer_Sheet_PlayUnit(cur_freqs, n, sheet->unit_ms);
                cur_count = n;
            }
            legato = false;
            p = q;
            continue;
        }

        uint16_t freq_hz = 0;
        if (Buzzer_KeyToFreq(ch, &freq_hz)) { /* 单音 */
            cur_freqs[0] = freq_hz;
            cur_count = 1;
            Buzzer_Sheet_PlayUnit(cur_freqs, 1, sheet->unit_ms);
        }
        /* 未绑定字符(如逗号)直接跳过，仅用于分隔便于阅读 */
        legato = false;
        p++;
    }

    Buzzer_Off();
    logPrintln("Sheet done: %s", sheet->name);
}

#endif /* !BUZZER_ACTIVE_MODE */

/**
 * @brief 乐谱 shell 命令
 * @param argc 参数个数
 * @param argv 参数列表
 */
static void Buzzer_Sheet_Tool(int argc, char *argv[]) {
#if BUZZER_ACTIVE_MODE
    (void)argc;
    (void)argv;
    logPrintln("Active buzzer: sheet not applicable");
#else
    Buzzer_EnsureInit();

    if (argc < 2) {
        logPrintln("Usage: sheet <name>   (play a sheet, ESC to abort)");
        logPrintln("Sheets:");
        for (uint32_t i = 0; i < BUZZER_SHEET_COUNT; i++) {
            logPrintln("  %s (%d ms/beat)", buzzer_sheets[i].name, buzzer_sheets[i].unit_ms);
        }
        return;
    }

    for (uint32_t i = 0; i < BUZZER_SHEET_COUNT; i++) {
        if (strcmp(argv[1], buzzer_sheets[i].name) == 0) {
            Buzzer_Sheet_Play(&buzzer_sheets[i]);
            return;
        }
    }

    logPrintln("Sheet not found: %s", argv[1]);
    logPrintln("Usage: sheet <name>");
#endif
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN,
                 sheet,
                 Buzzer_Sheet_Tool,
                 Buzzer Sheet Player);
