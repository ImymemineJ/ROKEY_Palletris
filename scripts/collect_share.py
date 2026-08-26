"""배포용 Collect — 원격 참조를 **전부 로컬로** 내려 자기완결 폴더를 만든다.

    cd ~/isaacsim && ./isaac-sim.sh --no-window \
        --exec ~/palletris_ws/scripts/collect_share.py

★ 왜 필요한가. GUI 로 얹은 배경 두 개가 **절대 `https://` 페이로드**다:

    /World/full_warehouse              .../Isaac/5.1/Isaac/Environments/Simple_Warehouse/full_warehouse.usd
    /World/PlatformLuggageCart_A01_01  .../DigitalTwin/Assets/Warehouse/Equipment/Carts/...

  기존 컨베이어들은 Collect 가 `./omniverse-content-...` 상대경로로 받아 놨는데
  이 둘만 원격이다. 그대로 보내면 받는 쪽은 **에러 없이 배경만 안 보인다** —
  인터넷이 없거나 프록시면 페이로드가 조용히 실패한다.

★ **원본 폴더에 다시 뜨지 않는다.** 자기 자신을 덮는다. 새 폴더로 뜬다.
  `skip_existing=True` 라 이미 로컬인 1.1 GB 는 다시 안 받고 새 것만 받는다.
"""
import asyncio
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import omni.kit.app                                            # noqa: E402
import _kitexit                                                # noqa: E402

SRC = os.environ.get("PL_STAGE",
                     "/home/rokey/isaac_scenes/Collected_test_3/test_3.usd")
DST = os.environ.get("PL_COLLECT_DIR",
                     "/home/rokey/isaac_scenes/Collected_test_3_share")

if os.path.abspath(DST) == os.path.dirname(os.path.abspath(SRC)):
    raise SystemExit("원본 폴더에 다시 뜨면 자기 자신을 덮는다: %s" % DST)

print("\n원본 %s\n대상 %s\n" % (SRC, DST), flush=True)

_st = {"done": False, "ok": False, "root": "", "last": -1}


def _progress(cur, total):
    pct = int(cur * 100 / total) if total else 0
    if pct // 5 != _st["last"] // 5:          # 5 % 마다만 찍는다
        _st["last"] = pct
        print("   %3d %%  (%d / %d)" % (pct, cur, total), flush=True)


async def _go():
    from omni.kit.usd.collect import Collector
    try:
        c = Collector(SRC, DST, skip_existing=True)
        ok, root = await c.collect(progress_callback=_progress)
        _st["ok"], _st["root"] = bool(ok), str(root or "")
    except Exception as e:
        import traceback
        traceback.print_exc()
        print("!! Collect 실패: %r" % (e,), flush=True)
    finally:
        _st["done"] = True


asyncio.ensure_future(_go())


def _tick(_e):
    if not _st["done"]:
        return
    try:
        _sub["h"].unsubscribe()
    except Exception:
        pass
    print("\n결과 %s\n루트 %s" % ("성공" if _st["ok"] else "실패", _st["root"]),
          flush=True)
    _kitexit.shutdown(0 if _st["ok"] else 1)


_sub = {"h": omni.kit.app.get_app().get_update_event_stream()
        .create_subscription_to_pop(_tick, name="palletris-collect")}
