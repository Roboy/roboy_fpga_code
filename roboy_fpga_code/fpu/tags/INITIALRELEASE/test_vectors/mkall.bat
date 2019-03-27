
rem -------------------- fasu round to nearest even

pg -q -r 0 -m 1 -p 0          -o rtne/fasu_pat0a.hex
pg -q -r 0 -m 2 -p 0          -o rtne/fasu_pat0b.hex

pg -q -r 0 -m 1 -p 1          -o rtne/fasu_pat1a.hex
pg -q -r 0 -m 2 -p 1          -o rtne/fasu_pat1b.hex

pg -q -r 0 -m 1 -p 2          -o rtne/fasu_pat2a.hex
pg -q -r 0 -m 2 -p 2          -o rtne/fasu_pat2b.hex

pg -q -r 0 -m 1 -n 199990 -ll -o rtne/fasu_lga.hex
pg -q -r 0 -m 2 -n 199990 -ll -o rtne/fasu_lgb.hex

pg -q -r 0 -m 1 -n 199990     -o rtne/fasu_sma.hex
pg -q -r 0 -m 2 -n 199990     -o rtne/fasu_smb.hex


rem -------------------- fasu round to zero

pg -q -r 3 -m 1 -p 0          -o rtzero/fasu_pat0a.hex
pg -q -r 3 -m 2 -p 0          -o rtzero/fasu_pat0b.hex

pg -q -r 3 -m 1 -p 1          -o rtzero/fasu_pat1a.hex
pg -q -r 3 -m 2 -p 1          -o rtzero/fasu_pat1b.hex

pg -q -r 3 -m 1 -p 2          -o rtzero/fasu_pat2a.hex
pg -q -r 3 -m 2 -p 2          -o rtzero/fasu_pat2b.hex

pg -q -r 3 -m 1 -n 199990 -ll -o rtzero/fasu_lga.hex
pg -q -r 3 -m 2 -n 199990 -ll -o rtzero/fasu_lgb.hex

pg -q -r 3 -m 1 -n 199990     -o rtzero/fasu_sma.hex
pg -q -r 3 -m 2 -n 199990     -o rtzero/fasu_smb.hex


rem -------------------- fasu round to inf + (up)

pg -q -r 2 -m 1 -p 0          -o rup/fasu_pat0a.hex
pg -q -r 2 -m 2 -p 0          -o rup/fasu_pat0b.hex

pg -q -r 2 -m 1 -p 1          -o rup/fasu_pat1a.hex
pg -q -r 2 -m 2 -p 1          -o rup/fasu_pat1b.hex

pg -q -r 2 -m 1 -p 2          -o rup/fasu_pat2a.hex
pg -q -r 2 -m 2 -p 2          -o rup/fasu_pat2b.hex

pg -q -r 2 -m 1 -n 199990 -ll -o rup/fasu_lga.hex
pg -q -r 2 -m 2 -n 199990 -ll -o rup/fasu_lgb.hex

pg -q -r 2 -m 1 -n 199990     -o rup/fasu_sma.hex
pg -q -r 2 -m 2 -n 199990     -o rup/fasu_smb.hex


rem -------------------- fasu round to inf - (down)

pg -q -r 1 -m 1 -p 0          -o rdown/fasu_pat0a.hex
pg -q -r 1 -m 2 -p 0          -o rdown/fasu_pat0b.hex

pg -q -r 1 -m 1 -p 1          -o rdown/fasu_pat1a.hex
pg -q -r 1 -m 2 -p 1          -o rdown/fasu_pat1b.hex

pg -q -r 1 -m 1 -p 2          -o rdown/fasu_pat2a.hex
pg -q -r 1 -m 2 -p 2          -o rdown/fasu_pat2b.hex

pg -q -r 1 -m 1 -n 199990 -ll -o rdown/fasu_lga.hex
pg -q -r 1 -m 2 -n 199990 -ll -o rdown/fasu_lgb.hex

pg -q -r 1 -m 1 -n 199990     -o rdown/fasu_sma.hex
pg -q -r 1 -m 2 -n 199990     -o rdown/fasu_smb.hex


rem -------------------- fmul round to nearest even

pg -q -r 0 -m 4 -p 0          -o rtne/fmul_pat0.hex

pg -q -r 0 -m 4 -p 1          -o rtne/fmul_pat1.hex

pg -q -r 0 -m 4 -p 2          -o rtne/fmul_pat2.hex

pg -q -r 0 -m 4 -n 199990 -ll -o rtne/fmul_lg.hex

pg -q -r 0 -m 4 -n 199990     -o rtne/fmul_sm.hex


rem -------------------- fmul round to zero

pg -q -r 3 -m 4 -p 0          -o rtzero/fmul_pat0.hex

pg -q -r 3 -m 4 -p 1          -o rtzero/fmul_pat1.hex

pg -q -r 3 -m 4 -p 2          -o rtzero/fmul_pat2.hex

pg -q -r 3 -m 4 -n 199990 -ll -o rtzero/fmul_lg.hex

pg -q -r 3 -m 4 -n 199990     -o rtzero/fmul_sm.hex



rem -------------------- fmul round to inf + (up)

pg -q -r 2 -m 4 -p 0          -o rup/fmul_pat0.hex

pg -q -r 2 -m 4 -p 1          -o rup/fmul_pat1.hex

pg -q -r 2 -m 4 -p 2          -o rup/fmul_pat2.hex

pg -q -r 2 -m 4 -n 199990 -ll -o rup/fmul_lg.hex

pg -q -r 2 -m 4 -n 199990     -o rup/fmul_sm.hex



rem -------------------- fmul round to inf - (down)

pg -q -r 1 -m 4 -p 0          -o rdown/fmul_pat0.hex

pg -q -r 1 -m 4 -p 1          -o rdown/fmul_pat1.hex

pg -q -r 1 -m 4 -p 2          -o rdown/fmul_pat2.hex

pg -q -r 1 -m 4 -n 199990 -ll -o rdown/fmul_lg.hex

pg -q -r 1 -m 4 -n 199990     -o rdown/fmul_sm.hex


rem -------------------- fdiv round to nearest even

pg -q -r 0 -m 8 -p 0          -o rtne/fdiv_pat0.hex

pg -q -r 0 -m 8 -p 1          -o rtne/fdiv_pat1.hex

pg -q -r 0 -m 8 -p 2          -o rtne/fdiv_pat2.hex

pg -q -r 0 -m 8 -n 199990 -ll -o rtne/fdiv_lg.hex

pg -q -r 0 -m 8 -n 199990     -o rtne/fdiv_sm.hex


rem -------------------- fdiv round to zero

pg -q -r 3 -m 8 -p 0          -o rtzero/fdiv_pat0.hex

pg -q -r 3 -m 8 -p 1          -o rtzero/fdiv_pat1.hex

pg -q -r 3 -m 8 -p 2          -o rtzero/fdiv_pat2.hex

pg -q -r 3 -m 8 -n 199990 -ll -o rtzero/fdiv_lg.hex

pg -q -r 3 -m 8 -n 199990     -o rtzero/fdiv_sm.hex



rem -------------------- fdiv round to inf + (up)

pg -q -r 2 -m 8 -p 0          -o rup/fdiv_pat0.hex

pg -q -r 2 -m 8 -p 1          -o rup/fdiv_pat1.hex

pg -q -r 2 -m 8 -p 2          -o rup/fdiv_pat2.hex

pg -q -r 2 -m 8 -n 199990 -ll -o rup/fdiv_lg.hex

pg -q -r 2 -m 8 -n 199990     -o rup/fdiv_sm.hex



rem -------------------- fdiv round to inf - (down)

pg -q -r 1 -m 8 -p 0          -o rdown/fdiv_pat0.hex

pg -q -r 1 -m 8 -p 1          -o rdown/fdiv_pat1.hex

pg -q -r 1 -m 8 -p 2          -o rdown/fdiv_pat2.hex

pg -q -r 1 -m 8 -n 199990 -ll -o rdown/fdiv_lg.hex

pg -q -r 1 -m 8 -n 199990     -o rdown/fdiv_sm.hex


rem -------------------- Combo Tests

pg -q -m 15 -R -n 499995 -ll    -o combo/fpu_combo1.hex

pg -q -m 15 -R -n 499995 -s 17  -o combo/fpu_combo2.hex
