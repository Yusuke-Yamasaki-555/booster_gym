# 出力ファイル形式とファイル名を設定
set terminal pngcairo enhanced font 'Verdana,10'
set output 'cmd_q_leg.png'

# グラフのタイトルとラベルを設定
set title "Booster T1 Joint Angles"
set xlabel "Step"
set ylabel "Angle (radians)"

# グリッドと凡例を表示
set grid
set key outside top center horizontal

# 各関節のデータをプロット
plot "cmd_q_log.dat" using 1 with lines title "Left Hip Pitch", \
      ""           using 2 with lines title "Left Hip Roll", \
      ""           using 3 with lines title "Left Hip Yaw", \
      ""           using 4 with lines title "Left Knee Pitch", \
      ""           using 5 with lines title "Left Ankle Pitch", \
      ""           using 6 with lines title "Left Ankle Roll", \
      ""           using 7 with lines title "Right Hip Pitch", \
      ""           using 8 with lines title "Right Hip Roll", \
      ""           using 9 with lines title "Right Hip Yaw", \
      ""           using 10 with lines title "Right Knee Pitch", \
      ""           using 11 with lines title "Right Ankle Pitch", \
      ""           using 12 with lines title "Right Ankle Roll"