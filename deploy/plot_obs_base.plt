# 出力ファイル形式とファイル名を設定
set terminal pngcairo enhanced font 'Verdana,10'
set output 'obs_base_ang_vel.png'

# グラフのタイトルとラベルを設定
set title "Booster T1 Obs Base Ang Vel"
set xlabel "Step"
set ylabel "Ang Vel (rad/s)"

# グリッドと凡例を表示
set grid
set key outside top center horizontal

# 各関節のデータをプロット
plot "obs_base_ang_vel_log.dat" using 1 with lines title "data 1", \
      ""           using 2 with lines title "data 2", \
      ""           using 3 with lines title "data 3"