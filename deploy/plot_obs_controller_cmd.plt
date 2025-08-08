# 出力ファイル形式とファイル名を設定
set terminal pngcairo enhanced font 'Verdana,10'
set output 'obs_controller_cmd.png'

# グラフのタイトルとラベルを設定
set title "Booster T1 Controller Cmd"
set xlabel "Step"
set ylabel "Lin Vel, Ang Vel (m/s, m/s, rad/s)"

# グリッドと凡例を表示
set grid
set key outside top center horizontal

# 各関節のデータをプロット
plot "obs_controller_cmd_log.dat" using 1 with lines title "vx", \
      ""           using 2 with lines title "vy", \
      ""           using 3 with lines title "vyaw"