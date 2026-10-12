"""펜 끝 위치(도구 TCP 오프셋) 교정: 펜 끝을 종이 위 한 점에 댄 채 자세만 바꿔 여러 번 저장한 TCP 자세로 펜 끝 오프셋(x,y,z)을 최소제곱으로 구한다.
로봇을 부르지 않는다(저장된 자세만 계산).

원리: 저장한 TCP 위치 p_i, 자세 R_i, 도구 좌표계 안의 펜 끝 오프셋 t 에 대해, 펜 끝은 항상 같은 점 c 에 있다:  p_i + R_i·t = c.
미지수 t(3), c(3)을 모든 자세의 식을 쌓아 최소제곱으로 푼다. 자세가 서로 충분히 달라야 풀린다.
"""
import math

import numpy as np

import paper_loop as L

MIN_POSES = 4
MAX_POSES = 12
MIN_SPREAD_DEG = 15.0       # 펜 축이 서로 이만큼은 달라야 길이를 풀 수 있다
NOISE_MM = 0.3              # 손으로 저장한 위치의 흔들림 추정(예상 오차 계산용)
MAX_SIGMA_MM = 10.0         # 예상 오차가 이보다 크면 거부
WARN_SIGMA_MM = 2.0
MAX_RESID_MM = 8.0          # 계산한 펜 끝이 한 점에서 이만큼 넘게 벗어나면 같은 점에 대지 않은 것으로 보고 거부
WARN_RESID_MM = 2.5


class TcpError(ValueError):
    """사용자에게 그대로 보여 줄 한국어 오류."""


def solve(poses):
    """poses: [[x,y,z,rx,ry,rz], ...] → {"offset": [x,y,z], "point": [x,y,z], "residual_mm": [...], "sigma_mm": [...], ...}"""
    n = len(poses)
    if n < MIN_POSES:
        raise TcpError(f"자세를 {MIN_POSES}번 이상 저장해 주세요(지금 {n}번).")
    Rs = [np.array(L.rot(*p[3:6])) for p in poses]
    P = [np.array(p[:3], float) for p in poses]
    axes = [R[:, 2] for R in Rs]
    spread = max(math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b))))) for a in axes for b in axes)
    if spread < MIN_SPREAD_DEG:
        raise TcpError(f"펜 기울기가 서로 거의 같아요(최대 {spread:.0f}°). 같은 점에 펜 끝을 댄 채 손목을 앞뒤·좌우로 {MIN_SPREAD_DEG:.0f}° 이상 다르게 기울여서 저장해 주세요.")
    A = np.vstack([np.hstack([R, -np.eye(3)]) for R in Rs])
    b = -np.hstack(P)
    sol, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    if rank < 6:
        raise TcpError("자세 변화가 부족해서 펜 끝 위치를 정할 수 없어요. 서로 다른 방향(앞·뒤·좌·우)으로 기울인 자세를 섞어 주세요.")
    t, c = sol[:3], sol[3:]
    tips = [p + R @ t for p, R in zip(P, Rs)]
    resid = [float(np.linalg.norm(q - c)) for q in tips]
    sigma = np.sqrt(np.diag(np.linalg.inv(A.T @ A)))[:3] * NOISE_MM
    if float(sigma.max()) > MAX_SIGMA_MM:
        raise TcpError(f"자세 변화가 부족해서 결과를 믿기 어려워요(예상 오차 ±{float(sigma.max()):.0f}mm). 더 크게, 여러 방향으로 기울여 다시 저장해 주세요.")
    if max(resid) > MAX_RESID_MM:
        raise TcpError(f"계산한 펜 끝이 한 점에서 최대 {max(resid):.1f}mm 벗어나요. 모든 자세에서 펜 끝이 같은 점에 닿았는지 확인하고, 어긋난 자세를 지운 뒤 다시 계산하세요.")
    warnings = []
    if t[2] < 0:
        warnings.append("펜 끝이 도구 +Z 반대쪽으로 나왔어요. 저장할 때 쓴 도구 설정이 모서리 저장 때와 같은지 확인하세요.")
    if float(sigma.max()) > WARN_SIGMA_MM:
        warnings.append(f"예상 오차가 ±{float(sigma.max()):.1f}mm 예요. 자세를 더 다르게 해서 더 저장하면 정확해져요.")
    if max(resid) > WARN_RESID_MM:
        warnings.append(f"자세별 펜 끝이 한 점에서 최대 {max(resid):.1f}mm 벗어나요(펜이 점에서 미끄러졌을 수 있어요).")
    return {"offset": [round(float(v), 2) for v in t], "point": [round(float(v), 2) for v in c], "residual_mm": [round(r, 2) for r in resid],
            "sigma_mm": [round(float(v), 2) for v in sigma], "spread_deg": round(spread, 1), "poses": n, "warnings": warnings}
