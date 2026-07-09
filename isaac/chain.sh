#!/usr/bin/env bash
# Full evidence chain: validation matrix -> showcase videos -> gated stages.
# Launch:  setsid nohup bash /root/chain.sh > /root/fullchain.log 2>&1 &
# Stop:    bash /root/stop_chain.sh
echo $$ > /root/chain.pid
rm -f /root/CHAIN_ALL_DONE
bash /root/run_matrix.sh /root/sortmaster_out/final_arb
bash /root/showcase.sh /root/sortmaster_out/showcase_arb
bash /root/run_gated_stages.sh /root/sortmaster_out/gated_arb
touch /root/CHAIN_ALL_DONE
echo "CHAIN ALL DONE"
