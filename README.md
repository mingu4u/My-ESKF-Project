# 2. 설계

## 0. ESKF 기본 설계 요소

- 센서 종류 : IMU (가속도계, 자이로, 지자계), 피토센서, GPS
- 알고리즘 설명
    1. k 시점의 IMU 센서 데이터를 상태 모델에 대입, Prediction Step을 통해 위치, 속도, 자세 데이터를 예측한다. (사전 추정)
    2. GPS 데이터가 업데이트 됐을 때는 GPS 및 모든 센서를 활용해 Update Step을 진행하며, 업데이트 되지 않았을 때는 GPS를 제외한 가속도계, 지자계, Pitot를 활용해 Update Step을 진행한다. (자이로 성분이 H 행렬에 없는 이유는 쿼터니언 성분을 통해 이미 Update Step에 자이로 데이터가 활용되기 때문.)
    3. Error 를 Reset 하고, 공분산을 다시 업데이트 한다.
    4. k+1 시점의 1단계를 다시 반복한다.
- 나의 ESKF 알고리즘 알고리즘 플로우 차트
    
![output](https://github.com/user-attachments/assets/237e6fcd-1f30-41c2-a1bc-f986653f006a)
    

## **1. 비선형 상태 변화 모델**

### **1.1 상태 변수**

상태 변수 $\mathbf{x}$는 다음과 같이 구성됩니다

![image](https://github.com/user-attachments/assets/cd31c40f-c0ce-4062-adbb-9536b8f09324)



### **1.2 피토센서를 포함한 비선형 상태 변화 식**
![image 1](https://github.com/user-attachments/assets/1a968b09-4503-4172-9a12-d207ba370ff0)
### (1) **위치 (𝐩)**:
![image 2](https://github.com/user-attachments/assets/6a9dd616-de63-4a48-9f4e-4a2d43492e0d)


### (2) **속도 (𝐯)**:
![image 3](https://github.com/user-attachments/assets/b8847c2d-9cbb-489b-9c9c-0d224b7f36b3)

![g_END](https://github.com/user-attachments/assets/134a22c3-0221-4c3f-befd-26842fbec6da)

### (3) **자세 (𝐪)**:

![image 5](https://github.com/user-attachments/assets/2b847323-aa21-4052-94ff-4fadd39a8df4)

### (4) **가속도계 바이어스 (𝐛ₐ)**:

![image 6](https://github.com/user-attachments/assets/7e94efbd-a8b1-4af9-8985-7be603b654e4)

### (5) **자이로스코프 바이어스 (𝐛ₒ)**:

![image 7](https://github.com/user-attachments/assets/ad3c1ef3-c95e-486c-b788-75ebdd735ffc)

### (6) **피토센서 바이어스 (bₚᵢₜₒₜ)**:

피토센서 바이어스는 랜덤 워크 모델로 가정

![image 8](https://github.com/user-attachments/assets/e6882507-71a4-4c30-8231-f4c5d16e09d7)

## **2. 상태오차 모델**

상태오차 벡터는 다음과 같이 정의됩니다:
![image 9](https://github.com/user-attachments/assets/2799782c-9131-4fe9-97b8-342dc0723b72)

![image 10](https://github.com/user-attachments/assets/3601407b-0225-4195-86e7-85fe2b54a0b4)

![image 11](https://github.com/user-attachments/assets/1af8b08b-4aed-4fe9-9f61-9f7e5cf2028b)

## **3. 비선형 모델의 선형화**

- 비선형 모델

![image 12](https://github.com/user-attachments/assets/d328cef3-78ec-41ac-87bd-f1ae9fa4ce98)

### **3.1 위치 오차 (δ𝐩)**

위치는 속도에 의해 변화하므로:

![image 13](https://github.com/user-attachments/assets/76c21b91-fb62-4a16-b9cd-1282bd582bc9)

### **3.2 속도 오차 ($\delta \mathbf{v}$)**

비선형 속도 변화 모델은:

![image 14](https://github.com/user-attachments/assets/9a71c799-e314-44e2-a92d-4ea5ec1a84c5)

![image 15](https://github.com/user-attachments/assets/2e9acb04-3a73-4d22-8c8a-34c6b656b2e9)

실제 값과 추정 값 간의 차이로 정의:

![image 16](https://github.com/user-attachments/assets/abea0f9d-b100-4e42-94e3-c5daddb46fb1)

대입 및 정리하면:

![image 17](https://github.com/user-attachments/assets/c0a6d73d-3765-4996-baaf-71ca0d1227da)

### **3.3 자세 오차 (δθ)**

자세 변화 모델은:

소오차 근사 ($R(q) ≈ R(q̂)(I − [δθ]_×$))를 적용하면:
![image 18](https://github.com/user-attachments/assets/206eec78-88a9-4c2d-826c-f3c4872d1e8c)

![image 19](https://github.com/user-attachments/assets/876b04cb-d35c-4b87-908f-dfa6e76d9881)

---

### **3.4 피토센서 바이어스 오차 ($δb_{pitot}$)**

피토센서 바이어스는 랜덤 워크를 따르므로:

![image 20](https://github.com/user-attachments/assets/e0ab1a5f-6cfc-4357-9248-38c3b6ecf482)

## **4. Prediction Step**

### **4.1 상태오차 전이 행렬** $\mathbf{F}$:

모든 상태오차 식을 행렬 형태로 정리하면, 상태오차 전이 행렬 F는 다음과 같습니다:

![image 21](https://github.com/user-attachments/assets/9687720b-b0dd-4d22-a1d7-b2f3e7a9fe40)

※ 이때,  a_imu와 omega_imu는 바이어스가 보정된 값이 아닌, raw_data 값. (왜냐? 이미 bias가 선형화 과정에서 상쇄되어 사라졌기 때문.)

### **4.2 공분산 행렬 사전 예측**:

![image 22](https://github.com/user-attachments/assets/412886ab-93c0-424c-ac48-a8d80d26be80)

![image 23](https://github.com/user-attachments/assets/990b9d28-657e-4158-9bb4-92691ab02466)

## **5. Update Step**

- 잔차 계산

![image 24](https://github.com/user-attachments/assets/d88a9b53-935d-4623-afc5-87901cb5b8fe)

![image 25](https://github.com/user-attachments/assets/3331b271-bfa3-4ac1-b9b1-a5b409e1b5ce)

### (1) 측정 모델:

각 센서의 측정 모델은 다음과 같이 정의됩니다:

- **가속도계**:

![image 26](https://github.com/user-attachments/assets/af7be3ea-7e13-4a58-9588-27b8adb734bc)

- **지자계**:

![image 27](https://github.com/user-attachments/assets/6c534d74-5555-46d5-a11e-132c94dba534)

- **GPS**:

![image 28](https://github.com/user-attachments/assets/a74f2eb5-41de-4158-9c1e-670db58eb0bd)

- **피토센서**:

![image 29](https://github.com/user-attachments/assets/58d1e3ff-958e-4a23-b63e-4ff18a08362b)

### (2) 측정 자코비안 행렬 $\mathbf{H}$:

![image 30](https://github.com/user-attachments/assets/32c6e198-d7ec-4573-a297-b6ec5eecedae)

![image 31](https://github.com/user-attachments/assets/41d8eab3-8612-402a-a945-17448674bb59)

![image 32](https://github.com/user-attachments/assets/d09311f8-a03d-4f8a-9ba2-b26a7d12440c)

![image 33](https://github.com/user-attachments/assets/ba363edc-3ac6-491d-bd03-7748c0a64aa8)
- 경상남도 사천시 기준 m_ref

![image 34](https://github.com/user-attachments/assets/05a28342-4a7e-4ea6-b3ac-4b971308aadf)

### (3) 칼만 이득:

![image 35](https://github.com/user-attachments/assets/eaa1f383-efc4-4090-959a-ee090f015188)

![image 36](https://github.com/user-attachments/assets/046f38e1-8e3c-4417-99b1-cb1689c02bfe)

### **(3)-1 가속도계**

가속도계 노이즈는 측정되는 가속도의 불확실성을 나타냅니다. 대각 원소는 각 축의 분산으로 구성됩니다:

![image 37](https://github.com/user-attachments/assets/2cd1d20e-f6da-4a88-ac9f-8320260fd1d8)

​

### **(3)-2 자이로스코프**

자이로스코프 노이즈는 각속도 측정의 불확실성을 나타냅니다:

![image 38](https://github.com/user-attachments/assets/49b5f785-85ef-4347-85d5-b247ba475986)

### **(3)-3 지자계**

지자계의 노이즈는 자기장 벡터 측정값의 불확실성을 나타냅니다:

![image 39](https://github.com/user-attachments/assets/24964688-6b8a-44f8-b63c-367bf2d80ecc)

### **(3)-4 GPS**

GPS는 위치와 속도를 측정하며, 다음과 같은 공분산 행렬을 갖습니다:

- **위치**:

![image 40](https://github.com/user-attachments/assets/92d0a51d-b3ef-4d0b-ad70-2c018f4ea1b1)

- **속도**:

![image 41](https://github.com/user-attachments/assets/fc9c561f-2fa3-47ad-abba-db68100f85d0)

### **(3)-5 피토센서**

피토센서 노이즈는 공기 속도 측정값의 불확실성을 나타냅니다:

![image 42](https://github.com/user-attachments/assets/ed7fed0d-a6ff-434b-aaed-de81c7aaceb6)

### **(3).6 전체 R\mathbf{R}R 행렬 구성**

전체 측정 노이즈 공분산 행렬 R\mathbf{R}R는 각 센서의 노이즈 공분산 행렬을 결합하여 다음과 같이 구성됩니다:

![image 43](https://github.com/user-attachments/assets/91d3f2a2-0d2c-462d-b317-545a3d954808)

### (4) 상태 업데이트:

![image 44](https://github.com/user-attachments/assets/1721bf7e-cb71-42e0-8704-2938d9a667c3)

![image 24](https://github.com/user-attachments/assets/ca0dd5e0-b32a-4a1f-8ba4-14fa793d6161)


### (5) 공분산 업데이트:

![image 45](https://github.com/user-attachments/assets/a27994c6-3160-4cb5-8ddc-8c176beec30d)

## **6. Error Reset**

![image 46](https://github.com/user-attachments/assets/8c081db7-778a-47e8-ba4c-cf1a85294eeb)

### (1) 자세 오차 재설정:

소오차 회전벡터 δ**θ**를 초기화하고 𝐪를 갱신:

![image 47](https://github.com/user-attachments/assets/0c691574-0395-4cb0-bcd7-5b816ff18ff0)

소오차 근사에서는, Exp(δ**θ**)는 

![image 48](https://github.com/user-attachments/assets/8fa7dddd-195a-4b21-a0ce-6ee49d9b6935)

로 근사됨.

이후 𝐪 정규화 수행 (q = q / ||q||)

![image 49](https://github.com/user-attachments/assets/747d58f7-eee8-46b1-93fe-6eb96d6ad380)

![image 50](https://github.com/user-attachments/assets/5ad105c7-ce16-4dd4-be7c-791744944b9a)

### (2) 공분산 조정:

Error Reset 후 공분산 행렬은 다음과 같이 갱신됩니다:

![image 51](https://github.com/user-attachments/assets/ac12d849-812a-4b96-886e-d98a8a7d6ec4)

![image 52](https://github.com/user-attachments/assets/b3f62fdd-32ca-4c17-9971-52059694dc3e)

![image 53](https://github.com/user-attachments/assets/928a7730-96c0-4918-bd41-996814d10a4d)









































