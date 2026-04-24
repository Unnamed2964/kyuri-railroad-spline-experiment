import { TwoPointStraightPadSolverPage } from './components/TwoPointStraightPadSolverPage';

export default function App() {
  return (
    <main className="page-shell app-shell">
      <section className="page-header app-header">
        <p className="eyebrow">railroad spline experiment</p>
        <h1>铁路缓和曲线实验</h1>
        <p className="lead">
          当前只保留“二点约束 + 首尾直线”模式：在给定两点、两端切线和已知 A 的前提下，允许核心 S-C-S 曲线前后各补一段非负直线，并实时搜索满足约束的最大可行半径 R。
        </p>
      </section>

      <TwoPointStraightPadSolverPage />
    </main>
  );
}