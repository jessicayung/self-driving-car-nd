for epoch in `seq 20`
do
    for filename in "m_000005" "mm_000010" "u_000090"
    do
        cp "runs/epoch_"$epoch"/u"$filename".png" "samples_per_epoch/e"$epoch"_"$filename".png"
    done
done
