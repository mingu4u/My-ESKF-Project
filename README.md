# My-ESKF-Project
Error State Kalman Filter 기반 측위 센서 퓨전



## 0. ESKF 기본 설계 요소

- 센서 종류 : IMU (가속도계, 자이로, 지자계), 피토센서, GPS
- 알고리즘 설명
    1. k 시점의 IMU 센서 데이터를 상태 모델에 대입, Prediction Step을 통해 위치, 속도, 자세 데이터를 예측한다. (사전 추정)
    2. GPS 데이터가 업데이트 됐을 때는 GPS 및 모든 센서를 활용해 Update Step을 진행하며, 업데이트 되지 않았을 때는 GPS를 제외한 가속도계, 지자계, Pitot를 활용해 Update Step을 진행한다. (자이로 성분이 H 행렬에 없는 이유는 쿼터니언 성분을 통해 이미 Update Step에 자이로 데이터가 활용되기 때문.)
    3. Error 를 Reset 하고, 공분산을 다시 업데이트 한다.
    4. k+1 시점의 1단계를 다시 반복한다.
- 나의 ESKF 알고리즘 알고리즘 플로우 차트
    
    ![output.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/1a115b3d-ed7b-4791-a853-acaca5ab2dcd/output.png)
    

## **1. 비선형 상태 변화 모델**

### **1.1 상태 변수**

상태 변수 $\mathbf{x}$는 다음과 같이 구성됩니다

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/e42a3a9c-5a97-46d6-b497-02506c6cfa68/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/6c63ca40-60bd-4b96-97ae-93c20e1dd1a4/image.png)

### **1.2 피토센서를 포함한 비선형 상태 변화 식**

### (1) **위치 (𝐩)**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/3a5dc11f-e3aa-40db-830e-8c235275caf5/image.png)

### (2) **속도 (𝐯)**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/d4a695a3-2954-4118-974c-d04f6b8097b3/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/394f0193-e781-4b8e-aa67-a5623025b386/image.png)

### (3) **자세 (𝐪)**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/a72ce189-fd9e-4572-862a-6294f91da73d/image.png)

### (4) **가속도계 바이어스 (𝐛ₐ)**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/66e89c65-d952-467d-9441-1176cc42989d/image.png)

### (5) **자이로스코프 바이어스 (𝐛ₒ)**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/02b6432a-a4fc-4188-8c26-cd365ddc23ce/image.png)

### (6) **피토센서 바이어스 (bₚᵢₜₒₜ)**:

피토센서 바이어스는 랜덤 워크 모델로 가정

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/c5eee0e7-8bb3-4348-ab12-6ad8268b7d6c/image.png)

## **2. 상태오차 모델**

상태오차 벡터는 다음과 같이 정의됩니다:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/0d827db8-6287-4b27-824c-422d87cf6187/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/38f77ae8-2fcf-43a1-8d22-3c7c33db01e9/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/072d95ec-41e2-4871-a0e8-ca7b96d589e8/image.png)

## **3. 비선형 모델의 선형화**

- 비선형 모델

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/12f67143-012e-4bf9-a051-39073cdc0e3f/image.png)

### **3.1 위치 오차 (δ𝐩)**

위치는 속도에 의해 변화하므로:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/4441f805-5c2c-4c00-8688-c7c9a4b341b9/image.png)

### **3.2 속도 오차 ($\delta \mathbf{v}$)**

비선형 속도 변화 모델은:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/8cdf4c96-c619-4663-8849-f6d81186eb3b/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/52401905-7810-450f-b198-48308e216a8d/image.png)

실제 값과 추정 값 간의 차이로 정의:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/ba0a7793-3a62-46bc-809c-1e7b03078218/image.png)

대입 및 정리하면:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/daacd8d3-d6ba-4bce-83d3-edb95e13a5ce/image.png)

### **3.3 자세 오차 (δθ)**

자세 변화 모델은:

소오차 근사 ($R(q) ≈ R(q̂)(I − [δθ]_×$))를 적용하면:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/712d042c-ac85-4fe0-a7c2-55dfb1ea9edb/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/86798b30-58ae-4614-889e-1a2173e05e3e/image.png)

---

### **3.4 피토센서 바이어스 오차 ($δb_{pitot}$)**

피토센서 바이어스는 랜덤 워크를 따르므로:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/ffe023ce-0fff-4ae9-904f-ef03c6428355/image.png)

## **4. Prediction Step**

### **4.1 상태오차 전이 행렬** $\mathbf{F}$:

모든 상태오차 식을 행렬 형태로 정리하면, 상태오차 전이 행렬 F는 다음과 같습니다:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/cbf7e23e-6a34-4714-a7d3-a026fe8f9d44/image.png)

※ 이때,  a_imu와 omega_imu는 바이어스가 보정된 값이 아닌, raw_data 값. (왜냐? 이미 bias가 선형화 과정에서 상쇄되어 사라졌기 때문.)

### **4.2 공분산 행렬 사전 예측**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/8e00cb58-8736-41c9-8c8a-015e0a5eb0a1/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/8cbdd9dc-a27c-4719-a50e-b36093457b7d/image.png)

## **5. Update Step**

- 잔차 계산

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/9e2be548-b6f2-43df-bcde-339ddedf48e2/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/920b1338-1e1a-42d8-895b-6b4d92449d35/image.png)

### (1) 측정 모델:

각 센서의 측정 모델은 다음과 같이 정의됩니다:

- **가속도계**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/1274a800-5623-4314-ab41-2a3554d27411/image.png)

- **지자계**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/aa3bb0fe-23f5-4e72-8d79-c157fc98a08a/image.png)

- **GPS**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/29a9c983-4323-4092-84a7-c42c68c04747/image.png)

- **피토센서**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/5cd1c613-e6bb-4dea-a9b9-8f84d6a60e9f/image.png)

### (2) 측정 자코비안 행렬 $\mathbf{H}$:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/7b0a8e92-3c08-45bf-95f0-8478c1d73875/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/f0a403e8-c8a7-4ef7-8418-6b2306bc5d5f/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/c78d795c-77a9-4022-816e-2790ae7d4e7d/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/9b399d95-fd7c-41bd-9c91-ad549437fe39/image.png)

- 경상남도 사천시 기준 m_ref

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/e2a6908f-dcdf-44db-9dfa-7380425c8a97/image.png)

### (3) 칼만 이득:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/70b45786-de8a-4c6f-a8d8-d249e0af3f26/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/734dacc9-66ce-4a03-b805-8fd18ecbd0c3/image.png)

### **(3)-1 가속도계**

가속도계 노이즈는 측정되는 가속도의 불확실성을 나타냅니다. 대각 원소는 각 축의 분산으로 구성됩니다:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/3040369a-16f1-40c8-9f6a-7c7f57736fba/image.png)

### **(3)-2 자이로스코프**

자이로스코프 노이즈는 각속도 측정의 불확실성을 나타냅니다:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/c3923d11-d5c7-4199-b4d3-e1b3cacb32d8/image.png)

### **(3)-3 지자계**

지자계의 노이즈는 자기장 벡터 측정값의 불확실성을 나타냅니다:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/fb17d055-bbda-4af9-a51a-9bbfbffc0379/image.png)

### **(3)-4 GPS**

GPS는 위치와 속도를 측정하며, 다음과 같은 공분산 행렬을 갖습니다:

- **위치**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/d1380231-0864-40ea-9b59-1c0dd8abb3f7/image.png)

- **속도**:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/afc072c0-e390-443a-b9a0-73f8f8804a45/image.png)

### **(3)-5 피토센서**

피토센서 노이즈는 공기 속도 측정값의 불확실성을 나타냅니다:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/0b11e483-3f1f-45ed-abd2-294a30e94e5c/image.png)

### **(3).6 전체 R\mathbf{R}R 행렬 구성**

전체 측정 노이즈 공분산 행렬 R\mathbf{R}R는 각 센서의 노이즈 공분산 행렬을 결합하여 다음과 같이 구성됩니다:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/69b6ecf8-8536-4adf-ad18-acb834afc638/image.png)

### (4) 상태 업데이트:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/ae27efe6-a977-4f94-99f5-8f380133d795/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/9e2be548-b6f2-43df-bcde-339ddedf48e2/image.png)

### (5) 공분산 업데이트:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/9af2f28a-e91a-48c7-a34d-a1550194f097/image.png)

## **6. Error Reset**

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/6188c27d-5e22-4bc0-9c7a-60569a3f90e5/image.png)

### (1) 자세 오차 재설정:

소오차 회전벡터 δ**θ**를 초기화하고 𝐪를 갱신:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/9b07546c-5774-46a1-878f-8aceeab9c187/image.png)

소오차 근사에서는, Exp(δ**θ**)는 

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/fa409d2b-b9ad-479f-a62d-7e13e436f956/image.png)

로 근사됨.

이후 𝐪 정규화 수행 (q = q / ||q||)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/fd1cf310-a957-4737-b84a-43cffa92a497/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/76bdaedb-a00c-4857-bdab-ce2614a6def6/image.png)

### (2) 공분산 조정:

Error Reset 후 공분산 행렬은 다음과 같이 갱신됩니다:

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/d8f739e7-7bf0-46d8-87f2-f3730090e002/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/60a61e36-1c1c-42ee-84ff-6309e583005b/image.png)

![image.png](https://prod-files-secure.s3.us-west-2.amazonaws.com/67ffb295-ce5b-4c15-85a4-ac12230f7e3a/90a648f0-5dcf-498d-a9a2-01d1eede0355/image.png)
