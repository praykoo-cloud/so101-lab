"""미션 공통 예외 (여러 파일이 같은 클래스를 써야 해서 따로 둔다)"""


class SafetyAbort(RuntimeError):
    """더 진행하면 위험하거나 기록을 믿을 수 없다 → 그 자리에서 멈추고 clear 를 내지 않는다"""
