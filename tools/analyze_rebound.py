"""与上位机共用的离线回弹分析入口。"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'PY_SRIPT/algo_monitor'))
from rebound_analysis import analyze_rebound

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('session',type=Path)
    parser.add_argument('--output',type=Path,help='指定尚不存在的输出文件夹；默认在测试目录内另存带时间编号的新报告')
    args=parser.parse_args()
    result=analyze_rebound(args.session,args.output)
    print(result['output_dir'])
    print('Completed conditions:',result['report']['completed_conditions'])
