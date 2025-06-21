# 3x3 Rubik’s Cube Solver Using Point Clouds

이 프로젝트는 사진 촬영만으로 3x3 루빅스 큐브의 상태를 자동으로 인식하고, 최적의 큐브 해법을 출력하는 시스템입니다.  
포인트 클라우드 복원(openMVG), 파이썬 기반 큐브 상태 분석, 그리고 두 단계 큐브 솔버 알고리즘(RubiksCube-TwophaseSolver)을 연동합니다.

---

## 📦 전체 시스템 구조

1. **포인트 클라우드 복원**  
   - [openMVG](https://github.com/openMVG/openMVG)로 큐브의 3D 포인트 클라우드 생성  
2. **큐브 상태 분석**  
   - 제공된 파이썬 코드(`mecapstone.py`)로 포인트 클라우드에서 큐브의 각 면 색상 정보 추출  
   - 큐브 상태를 cubestring(큐브 공식 문자열)으로 변환  
3. **큐브 해법 도출**  
   - [RubiksCube-TwophaseSolver](https://github.com/hkociemba/RubiksCube-TwophaseSolver)로 cubestring을 입력해 최적 해법 계산

---

## 🖥️ 설치 및 환경설정

### 1. 시스템 요구사항

- Python 3.8 이상
- Ubuntu 20.04+ 또는 Windows 10+ (Linux 권장)
- C++ 빌드 환경 (openMVG용)
- pip, git, cmake, build-essential 등

### 2. 필수 패키지 설치

#### Python 패키지
```pip install open3d scikit-learn numpy matplotlib```



#### typing-extensions 오류 방지
```pip install --upgrade typing-extensions```



#### 큐브 솔버 엔진
```pip install RubikTwoPhase```



#### openMVG 설치 (포인트 클라우드 복원)
```git clone --recursive https://github.com/openMVG/openMVG.git
mkdir openMVG_Build && cd openMVG_Build
cmake -DCMAKE_BUILD_TYPE=RELEASE ../openMVG/src/
make -j$(nproc)
sudo make install```


- Windows 사용자는 [공식 문서](https://openmvg.readthedocs.io/en/latest/BUILD/) 참고

---

## 📸 사용법

### 1. 큐브 촬영

1. 큐브를 한 바퀴 돌려가며 여러 각도에서 사진 촬영 (최소 5장, 한 면이 바닥에 닿게)
2. 큐브를 180도 뒤집어 다시 여러 각도에서 사진 촬영 (최소 5장, 반대면이 바닥에 닿게)
3. 각각의 이미지를 `bottom_images/`와 `top_images/` 폴더에 저장

### 2. 포인트 클라우드 생성 (openMVG)

아래 명령어를 각각의 폴더(`bottom_images/`, `top_images/`)에 대해 실행하세요.

1. 이미지 리스트 생성
```openMVG_main_SfMInit_ImageListing -i bottom_images/ -o bottom_matches/```

2. 특징점 추출
```openMVG_main_ComputeFeatures -i bottom_matches/sfm_data.json -o bottom_matches/```

3. 매칭
```openMVG_main_ComputeMatches -i bottom_matches/sfm_data.json -o bottom_matches/```

4. 3D 재구성
```openMVG_main_IncrementalSfM -i bottom_matches/sfm_data.json -m bottom_matches/ -o bottom_reconstruction/```



- `top_images/`도 동일하게 진행
- 결과로 `bottom_reconstruction/point_cloud.ply`, `top_reconstruction/point_cloud.ply` 생성

### 3. 큐브 상태 분석 및 cubestring 생성

#### 1) 입력 파일 준비
- `input_bottom.ply` ← `bottom_reconstruction/point_cloud.ply` 복사
- `input_top.ply` ← `top_reconstruction/point_cloud.ply` 복사

#### 2) 파이썬 코드 실행
```python mecapstone.py```

- 실행 결과로 큐브 상태 문자열(cubestring)이 출력됩니다.
- 예시:  
Cubestring: UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB


### 4. 큐브 해법 도출

#### 1) Python에서 cubestring 사용 예시
```from twophase.solver import solve

cubestring = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB" # 위에서 얻은 문자열
solution = solve(cubestring, 19, 20)
print(f"Solution: {solution}")```


- `solution`에는 큐브를 맞추는 최적의 공식이 담깁니다.

---

## 🗂️ 프로젝트 폴더 구조 예시
```
project/
├── bottom_images/ # 큐브 하단 이미지
├── top_images/ # 큐브 상단 이미지
├── bottom_reconstruction/ # openMVG 결과
├── top_reconstruction/
├── input_bottom.ply # 분석용 포인트 클라우드
├── input_top.ply
├── mecapstone.py # 큐브 상태 분석 코드
├── cube_state.txt # (선택) 큐브 상태 문자열 저장 파일
└── README.md
```
---

## ⚙️ 주요 원리 및 상세 설명

- **포인트 클라우드 복원**: openMVG로 두 번(정방향/역방향) 촬영한 이미지에서 각각 5면의 포인트 클라우드를 복원
- **바닥/윗면 제거**: RANSAC으로 바닥 평면을 검출 후 제거, DBSCAN으로 큐브 외부 점 제거
- **6면 합치기**: 두 포인트 클라우드를 ICP로 정렬·합성해 완전한 6면 큐브 복원
- **색상 추출**: 각 면의 9개 조각 중심점 좌표를 내분점 공식을 이용해 계산, 평균 RGB→HSV 변환 후 중앙색 기준으로 색상 분류
- **큐브 문자열 생성**: 54개 조각의 색상을 공식 순서대로 나열해 cubestring 생성
- **해법 도출**: cubestring을 RubiksCube-TwophaseSolver에 입력해 최적 해법 산출

---

## 🛠️ 문제 해결

- **AttributeError: module 'typing_extensions' has no attribute 'Generic'**
    - typing-extensions 업그레이드 필요  
      ```
      pip install --upgrade typing-extensions
      ```
- **openMVG 빌드 오류**
    - cmake 버전 업그레이드 필요  
      ```
      sudo apt-get remove cmake
      # 최신 cmake 설치
      ```

---

## 📚 참고 자료

- [openMVG 공식 문서](https://openmvg.readthedocs.io/)
- [RubiksCube-TwophaseSolver](https://github.com/hkociemba/RubiksCube-TwophaseSolver)
- [Open3D 문서](http://www.open3d.org/docs/release/)

---

## 💡 발전 방향

- NxN 큐브(4x4, 5x5 등)로 확장 가능
- 모바일 앱/웹으로 자동화 가능 (사진 촬영→해법 출력까지)
- 밝기 변화에 강인한 HSV 기반 색상 분류 적용

---


