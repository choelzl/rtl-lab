# Stimuli file format

One stimuli file per benchmark, in CSV format. Each row describes a single memory transaction. The columns are:

| Column         | Type      | Description                                                                                                                                                |
| -------------- | --------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `task_id`      | integer   | Id of the task within the benchmark (`0`, `1`, `2`, ...).                                                                                                  |
| `operand`      | integer   | Operand index. Numbered per direction (read or write), and resets to `0` for each task. Read operands are `0`–`2` (max 3), write operands `0`–`1` (max 2). |
| `compute_step` | integer   | Compute step within the task (`0`, `1`, `2`, ...). Transactions of the same task sharing a `compute_step` may execute in parallel.                         |
| `write_en`     | `0` / `1` | `0` for a read access, `1` for a write access.                                                                                                             |
| `address`      | integer   | Memory address of the transaction.                                                                                                                         |
