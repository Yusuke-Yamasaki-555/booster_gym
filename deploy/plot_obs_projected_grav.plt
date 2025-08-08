# 出力ファイル形式とファイル名を設定
set terminal pngcairo enhanced font 'Verdana,10'
set output 'obs_projected_gravity.png'

# グラフのタイトルとラベルを設定
set title "Booster T1 Obs Projected Gravity"
set xlabel "Step"
set ylabel "Acc? (m/s^2)"

# グリッドと凡例を表示
set grid
set key outside top center horizontal

# 各関節のデータをプロット
plot "obs_projected_gravity_log.dat" using 1 with lines title "data 1", \
      ""           using 2 with lines title "data 2", \
      ""           using 3 with lines title "data 3"