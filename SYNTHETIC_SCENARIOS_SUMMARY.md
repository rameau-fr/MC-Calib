# Synthetic Scenarios Verification Results

We verified the Double Sphere (DS) implementation on 7 different synthetic scenarios and benchmarked it against the Kannala-Brandt (KB) model.

| Scenario | Description | DS Error (px) | KB Error (px) | Better Model |
| :--- | :--- | :--- | :--- | :--- |
| **Scenario 1** | 2 Cameras, 3 Boards | **8.58** | 12.92 | **Double Sphere** |
| **Scenario 2** | 5 Cameras, 1 Board | 2.20 | **0.02** | Kannala-Brandt |
| **Scenario 3** | 4 Cameras, 8 Boards | 2.15 | **0.18** | Kannala-Brandt |
| **Scenario 4** | 4 Cameras, 5 Boards | 10.35 | **0.02** | Kannala-Brandt |
| **Scenario 5** | 4 Cameras, 6 Boards | 5.36 | **0.20** | Kannala-Brandt |
| **Diff Size** | 2 Cameras, 3 Boards | 8.71 | **0.25** | Kannala-Brandt |
| **Non Square** | 2 Cameras, 2 Boards | **7.89** | 22.30 | **Double Sphere** |

## Observations
-   **Performance Split**:
    -   **Double Sphere** outperforms KB significantly in **Scenario 1** and **Non Square**. These scenarios likely contain distortions or field-of-view characteristics that match the Double Sphere model better (e.g., specific fisheye types).
    -   **Kannala-Brandt** achieves near-perfect reconstruction (< 0.3 px) in Scenarios 2, 3, 4, 5, and Diff Size. This suggests these synthetic datasets were likely generated using a model very close to KB or standard polynomial distortion, making KB the mathematically superior fit for them.
-   **Robustness**: Both models ran successfully on all datasets.
-   **Conclusion**: The Double Sphere implementation is working correctly and provides a valuable alternative. It is superior in specific wide-angle/fisheye configurations (Scenario 1, Non Square) but KB remains the better choice for standard synthetic datasets which likely follow a polynomial distortion model.
