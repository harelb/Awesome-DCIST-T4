# Geo anchoring and offline tiles

Use the spark_env python (no ROS):  `P=/home/harel/environments/dcist/spark_env/bin/python3`

Anchor the Pennovation Hydra map to UTM (use `--min-status 2`: status 0 = plain GPS fixes
that are several metres off; status 2 = RTK-corrected):

    D=/home/harel/data/pennovation_2025/courtyard-alleyways
    $P anchor_from_gps.py --bag $D --topic /euclid/fix \
        --trajectory $D/output/hydra/backend/trajectory.csv \
        --out $D/output/geo_anchor.json --min-status 2

Writes geo_anchor.json, geo_anchor.png (diagnostic overlay) and geo_anchor_fixes.csv (fix cache).
Map -> UTM: `[E,N] = R(yaw) * [x,y] + t`. `z0` = median trajectory z minus 0.5 m (Spot body height).

Fetch satellite tiles (z15-19) into ~/.adt4_console/tiles/<layer>/{z}/{x}/{y}.jpg:

    $P fetch_tiles.py --anchor $D/output/geo_anchor.json \
        --trajectory $D/output/hydra/backend/trajectory.csv --margin-m 80 --zmin 15 --zmax 19   # Esri has no real z20 here (placeholder tiles)

From a GeoTIFF instead: `$P fetch_tiles.py --geotiff foo.tif --layer mysite --zmin 16 --zmax 21`.
